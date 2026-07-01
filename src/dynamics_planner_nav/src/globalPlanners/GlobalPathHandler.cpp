// GlobalPathHandler.cpp — /plan 处理逻辑实现(声明见 .hpp)
#include "GlobalPathHandler.hpp"
#include "LocalGoalGenerator.hpp"      // 状态相关 local goal 生成/获取
#include "../robot/Go2.hpp"
#include "../robot/Utility.hpp"        // transform_lg(world→base_link)

void GlobalPathHandler::processEmptyGlobalPath(Robot_config* robot) {
    // 占位：空路径目前仅切 BRAKE。后续若要做空路径专用策略(原地搜索/保持上次目标等)放这里。
    robot->setRobotState(Robot_config::BRAKE);
}

void GlobalPathHandler::processValidGlobalPath(Robot_config* robot,
                                               const nav_msgs::msg::Path::SharedPtr& msg) {
    std::lock_guard<std::mutex> lock(robot->path_goal_mutex_);

    robot->global_paths.clear();
    robot->global_paths_odom.clear();
    robot->global_paths.reserve(msg->poses.size());
    robot->global_paths_odom.reserve(msg->poses.size());

    // /plan 是 odom 帧。global_paths_odom 存原始 odom 坐标(DDP 系 planner 用，traj 也是 odom)；
    // global_paths 存【base_link 机体帧】——用 transform_lg 把每点按机器人当前 odom 位姿
    // world(odom)→base_link 变换(TEB 用)。
    const auto rp = robot->getPoseState();

    // 已到达 global goal：保持停止，不再用 /plan 重新激活 local_goal_received，
    // 否则 2Hz 的 /plan 会立刻把刚到达时清掉的标志重新置位，狗又动起来打转。
    const bool need_lookahead = !robot->global_goal_reached;

    // 第一遍：双坐标系入库(odom 原始 + base_link 机体帧)。local goal 的【生成/获取】已抽到
    // globalPlanners/LocalGoalGenerator，按状态用三套前瞻方程 + 弧长机制统一完成。
    for (const auto& pose : msg->poses) {
        const double px = pose.pose.position.x;
        const double py = pose.pose.position.y;
        auto body = transform_lg(px, py, rp.x_, rp.y_, rp.theta_);

        robot->global_paths_odom.push_back({px, py});      // odom 原始
        robot->global_paths.push_back(body);               // base_link
    }

    if (need_lookahead) {
        // 规划时域 Σdt(s)：动力学可达位移 = max_vel_x * 该时域，作前瞻距离的尺度。
        double horizon = 0.0;
        for (double dtv : robot->getTimeInterval()) horizon += dtv;

        lgoal::LocalGoalQuery q;
        q.robot_state    = static_cast<int>(robot->getRobotState());
        q.rx             = rp.x_;
        q.ry             = rp.y_;
        q.base_lookahead = robot->local_goal_distance;
        q.max_vel_x      = robot->max_vel_x;     // 当前状态前向速度上限(已随状态缩放)
        q.horizon_time   = horizon;

        const int idx = lgoal::generateLocalGoal(robot->global_paths_odom, q);
        if (idx >= 0) {
            robot->local_goal_odom = robot->global_paths_odom[idx];   // odom 原始
            robot->local_goal      = robot->global_paths[idx];        // base_link
            robot->local_goal_received = true;
        }

        // 收到有效全局路径、里程计就绪、且【已收到导航目标】时，才离开 INIT 进入正常规划。
        // self_ctrl 模式下 global_path_provider 没目标也持续发 /plan，若不要求 global_goal_received，
        // 开机没点目标就会 INIT→NORMAL(进而被速度状态机误判)。故必须等真正有目标才启动。
        if (rp.valid_ && robot->global_goal_received &&
            robot->getRobotState() == Robot_config::INIT) {
            robot->setRobotState(Robot_config::NORMAL);
        }
    } else {
        robot->local_goal_received = false;
    }

    // 每收到一条有效 /plan 就发布目标可视化(odom 帧)：local_goal 绿、global_goal 红。
    // 此处已持有 path_goal_mutex_，view_Goal 内部只发 Marker、不加锁，安全。
    robot->view_Goal(robot->local_goal_odom, robot->global_goal_odom);
}
