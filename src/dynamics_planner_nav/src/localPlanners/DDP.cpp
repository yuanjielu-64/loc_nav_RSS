#include "localPlanners/DDP.hpp"
#include "utils/Algebra.hpp"
#include "utils/Timer.hpp"
#include "localPlanners/CollisionChecking.hpp"

namespace Antipatrea {

    void DDP::updateRobotState() {
        // Thread-safe read: get snapshot of current state
        auto current_state = robot->getPoseState();
        timeInterval = robot->getTimeInterval();

        current_vel = current_state.vx_;

        parent = {current_state.x_, current_state.y_, current_state.theta_,
                  current_state.vx_, current_state.vy_, current_state.angular_velocity_, true};
        parent_odom = current_state;
    }

    bool DDP::Solve(int nrIters, double dt, bool &canBeSolved) {
        (void)nrIters;
        (void)canBeSolved;
        geometry_msgs::msg::Twist cmd_vel;

        if (!robot) return false;

        updateRobotState();

        std::pair<std::vector<PoseState>, bool> best_traj;
        best_traj.first.reserve(nr_steps_);

        commonParameters(*robot);
        switch (robot->getRobotState()) {
            case Robot_config::BLIND:
                return handleNoMapPlanning(cmd_vel);

            case Robot_config::NORMAL:
                return handleNormalSpeedPlanning(cmd_vel, best_traj, dt);

            case Robot_config::CAUTIOUS:
                return handleLowSpeedPlanning(cmd_vel, best_traj, dt);

            default:
                return handleAbnormalPlanning(cmd_vel, best_traj, dt);
        }
    }

    bool DDP::handleNoMapPlanning(geometry_msgs::msg::Twist &cmd_vel) {

        normalParameters(*robot);

        auto global_goal = robot->getGlobalGoalOdomCfg();
        if (global_goal.size() < 2) {
            publishCommand(*robot, cmd_vel, 0.0, 0.0);
            return true;
        }

        const double angle_to_goal = calculateTheta(parent, &global_goal[0]);

        double angular = std::clamp(angle_to_goal, -1.0, 1.0);
        angular = (angular > 0) ? std::max(angular, 0.1) : std::min(angular, -0.1);

        publishCommand(*robot, cmd_vel, robot->max_vel_x, angular);
        return true;
    }

    bool DDP::handleNormalSpeedPlanning(geometry_msgs::msg::Twist &cmd_vel,
                                        std::pair<std::vector<PoseState>, bool> &best_traj, double dt) {

        normalParameters(*robot);

        auto result = ddp_planning(parent, parent_odom, best_traj, dt);

        robot->viewTrajectories(best_traj.first, nr_steps_);

        // ddp_planning 失败(无任何无碰撞轨迹) → best_traj.first 为空。在 NORMAL 这通常意味着
        // 宽/快采样在当前窄处找不到解：与其原地急停、苦等速度状态机的 0.5s 低速迟滞，不如
        // 立刻降到 CAUTIOUS——其更慢、更密、带侧移 vy 的采样在窄道更可能成功。
        // 注意：NORMAL 还在速度上不能急停归零(对四足是冲击)，按【当前实测速度减半】平滑减速，
        // 逐帧衰减成下坡；下一帧即按 CAUTIOUS 重规划。(size<=3 也兼作访问 [3] 的越界防御。)
        if (best_traj.first.size() <= 3) {
            const auto cur = robot->getPoseState();
            publishCommand(*robot, cmd_vel,
                           cur.vx_ * 0.5, cur.vy_ * 0.5, cur.angular_velocity_ * 0.5);
            return true;
        }

        if (result == false) {
            publishCommand(*robot, cmd_vel, best_traj.first[3].vx_ / 2, best_traj.first[3].vy_ / 2, best_traj.first[3].angular_velocity_ / 2);
        } else
            publishCommand(*robot, cmd_vel, best_traj.first[3].vx_, best_traj.first[3].vy_, best_traj.first[3].angular_velocity_);

        return true;
    }

    bool DDP::handleLowSpeedPlanning(geometry_msgs::msg::Twist &cmd_vel,
                                     std::pair<std::vector<PoseState>, bool> &best_traj, double dt) {


        lowSpeedParameters(*robot);

        auto result = ddp_planning(parent, parent_odom, best_traj, dt);

        robot->viewTrajectories(best_traj.first, nr_steps_);

        if (!result || best_traj.first.size() <= 3) {
            publishCommand(*robot, cmd_vel, -0.1, 0.0, 0.0);
        } else
            publishCommand(*robot, cmd_vel, best_traj.first[3].vx_, best_traj.first[3].vy_, best_traj.first[3].angular_velocity_);

        return true;
    }

    bool DDP::handleAbnormalPlanning(geometry_msgs::msg::Twist &cmd_vel,
                                    std::pair<std::vector<PoseState>, bool> &best_traj, double dt) {

        switch (robot->getRobotState()) {

        case Robot_config::BRAKE: {
            double vel = robot->getPoseState().vx_;
            if (vel >= 0.3)
                publishCommand(*robot, cmd_vel, -0.5, 0.0);
            else if (vel >= 0.1 && vel <= 0.3)
                publishCommand(*robot, cmd_vel, -0.3, 0.0);
            else {
                publishCommand(*robot, cmd_vel, -0.1, 0.0);
                // 速度已刹到 0 附近 → 进入脱困前复位计数(本次脱困重新计时、去抖重新累计)，
                // 然后切 RECOVER 开始脱困。BRAKE 的负速度仅用于刹停消除前进惯性，非倒退意图。
                robot->recover_times = 0;
                robot->recover_to_low_count = 0;
                robot->push_stuck_count = 0;
                robot->push_min_force = 1e9;
                robot->push_active = false;
                robot->push_gaveup = false;
                robot->recover_phase = kRecoverRotate;   // 进入 RECOVER 从 Rotate(先对准)开始
                robot->recover_progress_init = false;     // 进度看门狗重新计量
                robot->recover_escape_active = false;     // 清掉残留的逃逸动作
                // 逐级加码：若距上次退出 RECOVER < 2s(短时间内又掉回来)，说明上次没真脱困，
                // recover_level++(成功阈值随之翻倍)；否则视为全新一次脱困，归 0。
                constexpr double kRecoverRelapseWindow = 2.0;   // 复发判定窗口(s)
                if ((robot->now() - robot->recover_exit_time).seconds() < kRecoverRelapseWindow)
                    ++robot->recover_level;
                else
                    robot->recover_level = 0;
                robot->setRobotState(Robot_config::RECOVER);
            }
            return true;
        }

        case Robot_config::RECOVER: {
            // 锁存式脱困：① 斥力安全网(永远最先) → ★进度看门狗/逃逸(死锁则倒退+转身换位姿) →
            //   ② Rotate 朝目标对准 → ③ Plan(锁存,不再判朝向)。
            // recoverParameters(robot_radius_<0, min_traj_length_=0)下 ddp 几乎总有解；ddp 恒有解
            // 但狗可能不动的死锁由★看门狗兜底，故 Plan 不设失败兜底。
            constexpr double kAlignThreshold = M_PI / 6;  // 朝向闸门(30°)：进 Plan 的一次性门槛

            const bool   has_goal      = (local_goal_odom.size() >= 2);
            const double angle_to_goal = has_goal ? calculateTheta(parent, &local_goal_odom[0]) : 0.0;

            // ① 斥力安全网 — 任何阶段最高优先：快撞了(<0.1)先解贴。解完/推不动都回 Rotate 重新对准。
            switch (repulsionPush(cmd_vel)) {
                case PushResult::Pushed:
                    robot->recover_phase = kRecoverRotate;    // 被顶出挪了位置 → 回 Rotate 重对准
                    return true;
                case PushResult::Stuck:
                    robot->recover_phase = kRecoverRotate;
                    rotateToGoal(cmd_vel,                        // 贴身又推不动 → 边朝 local goal 转、边沿斥力反方向顽头出去
                                 robot->push_escape_vx, robot->push_escape_vy);
                    return true;
                case PushResult::Safe:
                    break;                                     // 身体已安全 → 按当前阶段继续(Rotate/Plan)
            }

            // ★ 进度看门狗 + 逃逸(优先级仅次于斥力)：根治"recoverParameters 下 ddp 恒有解、
            //   但真路被堵→狗龟速/不动→速度型退出永不触发→死锁在 RECOVER"。用实测位移判死锁，
            //   触发"倒退+朝更空一侧转身"逃逸打破陷阱。只在 Plan 阶段计量(Rotate 原地转不算)。
            if (recoverEscape(cmd_vel)) return true;

            // ② Rotate 阶段 — 朝向偏太多先转正；对准(±30°)即【锁存】进 Plan，从此不再判朝向。
            if (robot->recover_phase == kRecoverRotate) {
                if (has_goal && std::fabs(angle_to_goal) > kAlignThreshold) {
                    RCLCPP_INFO_THROTTLE(robot->get_logger(), *robot->get_clock(), 300,
                        "[RECOVER] ②Rotate对准 angle=%.0f°", angle_to_goal * 180.0 / M_PI);
                    rotateToGoal(cmd_vel);
                    return true;
                }
                robot->recover_phase = kRecoverPlan;           // ★锁存：进 Plan
            }

            // ③ Plan 阶段(锁存) — 只跑 ddp 朝目标前进；即使 ddp 把朝向带过 30° 也不拽回
            //   (靠 ddp 自身 ori_cost 收敛朝向)，从而根除"成功却被朝向闸门拉回"的左右摇摆。
            //   退出 RECOVER 不在这里判：交给状态机按"实测 |vx|≥0.1×bridge 持续 1s"切 CAUTIOUS
            //   (见 Go2_stateMachine.cpp)，避免 recoverParameters 下 ddp 恒有解导致廉价退出。
            recoverParameters(*robot);
            const bool solved = ddp_planning(parent, parent_odom, best_traj, dt);
            robot->viewTrajectories(best_traj.first, nr_steps_);
            if (solved && best_traj.first.size() > 3) {
                RCLCPP_INFO_THROTTLE(robot->get_logger(), *robot->get_clock(), 300,
                    "[RECOVER] ③Plan前进 vx=%.2f vy=%.2f",
                    best_traj.first[3].vx_, best_traj.first[3].vy_);
                publishCommand(*robot, cmd_vel,
                               best_traj.first[3].vx_, best_traj.first[3].vy_,
                               best_traj.first[3].angular_velocity_);
                return true;
            }

            // 防御：recoverParameters 下 ddp 极罕见地无解 → 原地停一拍，留在 Plan 下帧重试，
            // 不回 Rotate(避免破坏 Plan 锁存语义、重新引入摇摆)。
            publishCommand(*robot, cmd_vel, 0.0, 0.0, 0.0);
            return true;
        }

        case Robot_config::ROTATE:
            return true;     // TODO: 暂未实现


        default:
            return true;
        }
    }

    // publishCommand 已统一到 LocalPlannerBase(static)，DDP 不再保留自己的副本。

    DDP::PushResult DDP::repulsionPush(geometry_msgs::msg::Twist &cmd_vel) {
        constexpr double kTriggerDist = 0.05;  // 触发线(m)：任一方向 < 此值(贴脸/侵入)才【启动】斥力
        constexpr double kPushDist    = 0.1;   // 推开目标(m)：推到所有方向 ≥ 此值就收手(迟滞带 0.05~0.1)
        constexpr double kBodySafe    = 0.05;  // 身体安全线(m)：推到物理极限(stuck)但 min_clear ≥ 此值=没真贴障 →
                                               //   接受当前位置交给 Plan，并标记本片窄区"推不开"，不再空转重推
        constexpr double kPushGain    = 1.5;   // 合力模长 → 顶出速度 增益（贴得越紧推得越猛）
        constexpr double kPushMin     = 0.35;  // 最小顶出速度(m/s)：刚够冲过 sport 死区，又不至于猛推
        constexpr double kCrawlRatio  = 0.45;  // 顶出速度上限 = 桥速度 × 此比例
        constexpr int    kStuckLimit  = 10;    // hard_mag 连续不创新低超此帧数 → 判定推不动(Stuck)
        constexpr double kProgressEps = 0.02;  // hard_mag 创新低至少小这么多才算"推得动"(滤噪)
        constexpr double kGoalAttract = 0.6;   // goal 门控引力强度：0=纯朝最空、越大越偏向 local goal 那侧

        // 读 8 方向最小余量(触发判据) + 算朝 kPushDist 的顶出合力(weight = kPushDist − 余量)。
        std::array<double, Robot_config::DIR_SECTOR_COUNT> clearance;
        robot->getDirectionClearance(clearance);
        double min_clear = std::numeric_limits<double>::infinity();
        for (double c : clearance) if (c < min_clear) min_clear = c;

        double push_x, push_y;
        const double hard_mag = robot->computeHardRepulsion(kPushDist, push_x, push_y);

        // 出了窄区(已推/走到 ≥ kPushDist 开阔) → 清"推不开"标记，下次贴近可重新推到 0.3。
        if (min_clear >= kPushDist) robot->push_gaveup = false;

        // 迟滞触发：未在斥力中时——
        if (!robot->push_active) {
            if (min_clear >= kTriggerDist) return PushResult::Safe;   // 本就够开阔
            // 已判定本片窄区"推不到 0.3"且身体仍安全 → 不再重复空推，交给 Plan(ddp 在小余量里能走)。
            if (robot->push_gaveup && min_clear >= kBodySafe) return PushResult::Safe;
            robot->push_active = true;             // 触发！进入斥力(尽量推到所有方向 ≥0.3)
            robot->push_stuck_count = 0;
            robot->push_min_force = 1e9;
        }

        // 斥力中：推到所有方向 ≥ kPushDist(hard_mag≈0) 才算推开完成 → 退出斥力。
        if (hard_mag <= 1e-3) {
            robot->push_active = false;
            robot->push_stuck_count = 0;
            robot->push_min_force = 1e9;
            return PushResult::Safe;
        }

        // 转身时本计数【不清零】，所以推不动会持续转、不重数。
        if (hard_mag < robot->push_min_force - kProgressEps) {
            robot->push_min_force = hard_mag;
            robot->push_stuck_count = 0;
        } else {
            ++robot->push_stuck_count;
        }

        if (robot->push_stuck_count > kStuckLimit) {
            // 已推到物理极限(再推 hard_mag 不降)：min_clear ≥ kBodySafe 说明身体没真被侵入 →
            //   接受这个"已尽量推开"的位置，标记本片窄区推不开，交给 Plan 让 ddp 在小余量里前进。
            if (min_clear >= kBodySafe) {
                robot->push_active = false;
                robot->push_stuck_count = 0;
                robot->push_min_force = 1e9;
                robot->push_gaveup = true;
                RCLCPP_INFO_THROTTLE(robot->get_logger(), *robot->get_clock(), 300,
                    "[RECOVER] ①已尽量推开(余量=%.2f,推不到%.2f) → 交给 Plan", min_clear, kPushDist);
                return PushResult::Safe;
            }
            // 真贴/侵入(min_clear < kBodySafe)却推不动 → 边转身破楔、边沿斥力反方向把头顶出去。
            //   侵入时方向用纯斥力反方向(push_x/push_y 已是"离开障碍"方向)，速度同推开逻辑，
            //   存入成员供 rotateToGoal 叠加到 w 上(边转边顶，二者不再互斥)。
            {
                const double inv = 1.0 / hard_mag;
                double push_speed = std::max(kPushMin, hard_mag * kPushGain);
                push_speed = std::min(push_speed, robot->bridge_max_velocity_ * kCrawlRatio);
                robot->push_escape_vx = push_x * inv * push_speed;
                robot->push_escape_vy = push_y * inv * push_speed;
            }
            RCLCPP_INFO_THROTTLE(robot->get_logger(), *robot->get_clock(), 300,
                "[RECOVER] ①推不动仍贴障 → 边转边顶(hard_mag=%.2f 余量=%.2f 顶出vx=%.2f vy=%.2f)",
                hard_mag, min_clear, robot->push_escape_vx, robot->push_escape_vy);
            return PushResult::Stuck;
        }

        // 还推得动 → 朝【开阔度加权 + goal 门控引力】方向挤出(方向不再用斥力合力，
        //   避免走廊里两侧平行墙的斜后扇区污染方向；hard_mag 仍只作触发/贴障与 stuck 判据)。
        const bool   has_goal   = (local_goal_odom.size() >= 2);
        const double goal_angle = has_goal ? calculateTheta(parent, &local_goal_odom[0]) : 0.0;
        double esc_x, esc_y;
        const double esc_mag = robot->computeEscapeDirection(goal_angle, kGoalAttract, esc_x, esc_y);
        // 兜底：逃逸方向算不出(四周全未观测) → 退回斥力反方向(老方法)。
        if (esc_mag < 1e-6) {
            const double inv = 1.0 / hard_mag;
            esc_x = push_x * inv;
            esc_y = push_y * inv;
        }
        robot->viewRepulsion(esc_x, esc_y);   // 可视化改为展示实际逃逸方向

        double push_speed = std::max(kPushMin, hard_mag * kPushGain);
        push_speed = std::min(push_speed, robot->bridge_max_velocity_ * kCrawlRatio);
        RCLCPP_INFO_THROTTLE(robot->get_logger(), *robot->get_clock(), 300,
            "[RECOVER] ①斥力推开 hard_mag=%.2f 最小余量=%.2f stuck=%d 速度=%.2f 方向=%.0f°",
            hard_mag, min_clear, robot->push_stuck_count, push_speed,
            std::atan2(esc_y, esc_x) * 180.0 / M_PI);
        publishCommand(*robot, cmd_vel, esc_x * push_speed, esc_y * push_speed, 0.0);
        return PushResult::Pushed;
    }

    void DDP::rotateToGoal(geometry_msgs::msg::Twist &cmd_vel,
                           double add_vx, double add_vy) {
        const bool   has_goal = (local_goal_odom.size() >= 2);
        const double angle    = has_goal ? calculateTheta(parent, &local_goal_odom[0]) : 0.0;
        double rot = has_goal ? std::clamp(angle, -1.5, 1.5) : 0.8;
        rot = (rot >= 0.0) ? std::max(rot, 0.5) : std::min(rot, -0.5);  // 最小转速防停滞(够快才转得动)
        robot->recover_to_low_count = 0;                                // 没前进 → 清零成功计数
        publishCommand(*robot, cmd_vel, add_vx, add_vy, rot);           // add_vx/vy: 边转边顶的斥力反方向顶出
    }

    bool DDP::recoverEscape(geometry_msgs::msg::Twist &cmd_vel) {
        // 解死锁：recoverParameters 下 ddp 恒有解，狗却可能龟速/不动 → 速度型退出永不触发，
        // 卡死在 RECOVER。这里用【实测位移】当真值：一段时间内没真正挪动 → 判死锁，执行
        // 【倒退 + 朝更空一侧原地转】逃逸。逃逸【逐帧】发指令、靠实测反馈结束(不用固定时长)，
        // 且本函数在斥力安全网【之后】调用 → 每帧都先过斥力，逃逸天然让位于避撞。
        constexpr double kProgressDist    = 0.15;  // 滑动窗口内实测位移 ≥ 此值 = 真在动(重置窗口)
        constexpr double kProgressTimeout = 4.0;   // 持续位移不足这么久(s) = 判死锁
        constexpr double kEscapeBackVx    = -0.15; // 逃逸倒退速度(m/s，负=后退离开堵点)
        constexpr double kEscapeSpin      = 0.8;   // 逃逸原地转速(rad/s)
        constexpr double kEscapeTurn      = M_PI / 3;  // 逃逸结束条件①：累计转过此角(≈60°)
        constexpr double kEscapeMove      = 0.20;      // 逃逸结束条件②：累计挪动此距离(m)

        const rclcpp::Time t_now = robot->now();
        const double cx = parent.x_, cy = parent.y_, cth = parent.theta_;

        // (a) 逃逸中：每帧只发一拍"倒退+转"，靠实测【转角/位移】判完成(先到为准)，不数时长。
        //     注意斥力已在调用本函数前先行；若斥力本帧接管则根本到不了这里，故逃逸不会压过避撞。
        if (robot->recover_escape_active) {
            // 逃逸目标转角随 recover_level 逐级加大(越反复卡死、扫得越宽)，上限 180°。
            const double turn_target = std::min(kEscapeTurn * (1.0 + 0.5 * robot->recover_level), M_PI);
            const double turned = std::fabs(normalizeAngle(cth - robot->recover_escape_theta));
            const double moved  = std::hypot(cx - robot->recover_progress_x,
                                             cy - robot->recover_progress_y);
            if (turned >= turn_target || moved >= kEscapeMove) {
                robot->recover_escape_active = false;
                // ★关键：换好位姿后【直接进 Plan，不回 Rotate】。否则 Rotate 会立刻把车头转回 goal，
                //   抵消逃逸刚转出的新朝向 → "转出去又被拽回"原地打转。让 ddp 的 ori_cost 平滑收敛朝向即可。
                robot->recover_phase = kRecoverPlan;
                robot->recover_progress_x = cx;
                robot->recover_progress_y = cy;
                robot->recover_progress_time = t_now;
                robot->recover_progress_init = true;
                return false;   // 交还本帧给 Plan
            }
            publishCommand(*robot, cmd_vel, kEscapeBackVx, 0.0,
                           robot->recover_escape_dir * kEscapeSpin);
            return true;
        }

        // (b) 死锁检测只在 Plan 阶段做(Rotate 原地转不产生位移，会误判)。非 Plan 时重置窗口，
        //     使 Plan 一进入就从当前位置重新计量。
        if (robot->recover_phase != kRecoverPlan) {
            robot->recover_progress_init = false;
            return false;
        }

        // (c) 看门狗首帧 / 滑动窗口初始化。
        if (!robot->recover_progress_init) {
            robot->recover_progress_x = cx;
            robot->recover_progress_y = cy;
            robot->recover_progress_time = t_now;
            robot->recover_progress_init = true;
            return false;
        }

        // (d) 实测位移够大 = 真在动 → 滑动窗口前移，不算卡。
        const double moved = std::hypot(cx - robot->recover_progress_x,
                                        cy - robot->recover_progress_y);
        if (moved >= kProgressDist) {
            robot->recover_progress_x = cx;
            robot->recover_progress_y = cy;
            robot->recover_progress_time = t_now;
            return false;
        }

        // (e) 位移长时间不足 → 判死锁，进入逃逸：升级加码 + 朝余量更大一侧转，并记录起始位姿。
        if ((t_now - robot->recover_progress_time).seconds() >= kProgressTimeout) {
            ++robot->recover_level;   // 逐次卡死 → 逐级加码(让原本的"死变量"真正生效)
            std::array<double, Robot_config::DIR_SECTOR_COUNT> clr;
            robot->getDirectionClearance(clr);
            // 左侧扇区(左前/左/左后) vs 右侧(右前/右/右后)余量之和，朝更空的一侧转。
            const double left  = clr[Robot_config::DIR_FRONT_LEFT]
                               + clr[Robot_config::DIR_LEFT]
                               + clr[Robot_config::DIR_BACK_LEFT];
            const double right = clr[Robot_config::DIR_FRONT_RIGHT]
                               + clr[Robot_config::DIR_RIGHT]
                               + clr[Robot_config::DIR_BACK_RIGHT];
            robot->recover_escape_dir = (left >= right) ? 1.0 : -1.0;
            robot->recover_escape_active = true;
            robot->recover_escape_theta  = cth;   // 逃逸起始朝向(算累计转角)
            robot->recover_progress_x = cx;        // 逃逸起始位置(算累计位移)
            robot->recover_progress_y = cy;
            RCLCPP_WARN(robot->get_logger(),
                "[RECOVER] 死锁(%.1fs 实测位移<%.2fm) → 逃逸:倒退+%s转 level=%d",
                kProgressTimeout, kProgressDist,
                robot->recover_escape_dir > 0 ? "左" : "右", robot->recover_level);
            publishCommand(*robot, cmd_vel, kEscapeBackVx, 0.0,
                           robot->recover_escape_dir * kEscapeSpin);
            return true;
        }

        return false;
    }

    bool DDP::ddp_planning(PoseState &state, PoseState &state_odom,
                            std::pair<std::vector<PoseState>, bool> &best_traj, double dt) {
        Timer::Clock d_t;
        Timer::Start(d_t);

        best_traj.first.reserve(nr_steps_);


        // 一次 solve 取一次真实外包模型(go2::Footprint)：选 volume_index_ 那一档，
        // 缓存供多线程只读。外层 !models.empty() 是防崩溃门(空时 clamp/索引会 UB)，必须保留。
        auto models = robot->getDynamicVolumes();
        if (!models.empty()) {
            int idx = std::clamp(volume_index_, 0, static_cast<int>(models.size()) - 1);
            solve_volume_ = models[idx];
            solve_volume_circ_r_ = solve_volume_.circumscribedRadius();
        }

        // ===== 障碍预处理(每帧一次，8 线程只读共享) =====
        // (A) 取一次 getDataMap()，扁平化成连续数组：消除每条候选轨迹深拷贝 vector<vector<double>>
        //     的隐藏开销(800×550 次堆操作)，并让 calc_obs_cost 的遍历 cache 友好。
        // (B) per-solve 可达预筛：800 条候选都从同一起点 state 出发，轨迹任一点离起点的
        //     位移上界 = Σ(max_vel_x · dt)。障碍若离起点比 "最大前伸 + 外接圆半径 + 影响半径"
        //     还远，则任何轨迹点都进不了影响半径 → 该障碍对本帧所有轨迹代价均为 0 → 直接剔除。
        {
            auto obss = robot->getDataMap();   // 仅此一次拷贝
            double max_reach = solve_volume_circ_r_ + obs_influence_dist_ + robot_radius_;
            for (double dtv : timeInterval) max_reach += robot->max_vel_x * dtv;
            const double reach2 = max_reach * max_reach;
            const double sx = state.x_, sy = state.y_;

            solve_obs_x_.clear();
            solve_obs_y_.clear();
            solve_obs_x_.reserve(obss.size());
            solve_obs_y_.reserve(obss.size());
            for (const auto &o : obss) {
                if (o.size() < 2) continue;
                const double ddx = o[0] - sx, ddy = o[1] - sy;
                if (ddx * ddx + ddy * ddy > reach2) continue;   // 够不到，剔除
                solve_obs_x_.push_back(static_cast<float>(o[0]));
                solve_obs_y_.push_back(static_cast<float>(o[1]));
            }
        }

        Window dw = calc_dynamic_window(*robot, state, dt);

        num_threads = robot->num_threads;

        std::vector<std::vector<Cost> > thread_costs(num_threads);
        std::vector<std::vector<std::vector<Control> > > thread_pairs(num_threads);
        std::vector<int> thread_processed(num_threads, 0);   // 各线程实际处理的轨迹数

        int task_per_thread = nr_pairs_ / num_threads;
        nr_pairs_ = task_per_thread * num_threads;

        std::vector<Control> pairs;
        pairs.reserve(nr_pairs_);

        // 三种状态的采样区别只在"网格样本数 + 末尾追加的固定样本"，网格生成逻辑完全相同，
        // 抽成一个 lambda：铺 count 个动态窗口网格样本（每 20 个掺一个上次最优解）。
        auto appendGrid = [&](int count) {
            int v_steps = static_cast<int>(std::sqrt(static_cast<double>(count)));
            int w_steps = (count + v_steps - 1) / v_steps;
            double v_step = (dw.max_velocity_ - dw.min_velocity_) / std::max(1, v_steps - 1);
            double w_step = (dw.max_angular_velocity_ - dw.min_angular_velocity_) / std::max(1, w_steps - 1);
            for (int i = 0; i < count; ++i) {
                double lv, av, lvy;
                if (i % 20 == 0 && delta_v_sum != FLT_MIN && delta_w_sum != FLT_MAX) {
                    lv = delta_v_sum;
                    av = delta_w_sum;
                    lvy = delta_vy_sum;
                } else {
                    lv = dw.min_velocity_ + (i % v_steps) * v_step;
                    av = dw.min_angular_velocity_ + ((i / v_steps) % w_steps) * w_step;
                    lvy = 0.0;
                }
                pairs.push_back({lv, lvy, av});
            }
        };

        const auto st = robot->getRobotState();

        if (st == Robot_config::CAUTIOUS) {
            // CAUTIOUS 种子(共 25)：慢进为主，外加少量【前向带轻微侧挪】，
            // 给窄道"侧身挤过"专门候选(不靠 vy 噪声碰)。Control = {vx, vy, w}。
            appendGrid(nr_pairs_ - 25);
            for (int i = 0; i < 15; ++i) pairs.push_back({0.2,  0.0,  0.0});   // 慢进
            for (int i = 0; i < 5;  ++i) pairs.push_back({0.15, 0.1,  0.0});   // 左前侧挪
            for (int i = 0; i < 5;  ++i) pairs.push_back({0.15, -0.1, 0.0});   // 右前侧挪
        } else if (st == Robot_config::RECOVER) {
            // RECOVER 种子(共 75)：脱困放开横移，专门加【纯横移螃蟹步左/右】种子，
            // 让侧移脱困有确定候选，而不是只靠 vy 高斯噪声碰。Control = {vx, vy, w}。
            appendGrid(nr_pairs_ - 75);
            for (int i = 0; i < 15; ++i) pairs.push_back({0.05, 0.0,   0.0});   // 慢进
            for (int i = 0; i < 15; ++i) pairs.push_back({0.02, 0.0,  -0.5});   // 右转
            for (int i = 0; i < 15; ++i) pairs.push_back({0.02, 0.0,   0.5});   // 左转
            for (int i = 0; i < 15; ++i) pairs.push_back({0.0,  0.25,  0.0});   // 纯左横移(螃蟹步)
            for (int i = 0; i < 15; ++i) pairs.push_back({0.0, -0.25,  0.0});   // 纯右横移(螃蟹步)
        } else {
            appendGrid(nr_pairs_);
        }

        std::vector<std::thread> threads;
        threads.reserve(num_threads);

        // 诊断计数清零(并行段前)：本帧统计碰撞/太短两类淘汰。
        reject_collision_.store(0, std::memory_order_relaxed);
        reject_too_short_.store(0, std::memory_order_relaxed);

        // 本帧并行评估的截止时刻 = 现在 + 时间预算。超过即令各线程提前停止(timeout_flag)，
        // 用已算出的候选选解，保证主循环维持 20Hz。每帧都要复位 timeout_flag。
        timeout_flag.store(false, std::memory_order_relaxed);
        solve_deadline_ = std::chrono::high_resolution_clock::now() +
                          std::chrono::microseconds(static_cast<long>(solve_time_budget_ms_ * 1000.0));

        for (int i = 0; i < num_threads; ++i) {
            int start = i * task_per_thread;
            int end = (i == num_threads - 1) ? nr_pairs_ : (start + task_per_thread);

            thread_costs[i].reserve(end - start);
            thread_pairs[i].reserve(end - start);

            threads.emplace_back(
                    [this, i, start, end, &state, &state_odom, &dw, &pairs,
                            &thread_costs, &thread_pairs, &thread_processed]() {
                        this->process_segment(i, start, end, state, state_odom, dw, pairs,
                                              thread_costs[i], thread_pairs[i],
                                              thread_processed[i]);
                    });
        }
        
        for (auto &thread: threads) {
            thread.join();
        }

        std::vector<Cost> costs;
        std::vector<std::vector<Control> > pairs_set;

        for (int i = 0; i < num_threads; ++i) {
            costs.insert(costs.end(), thread_costs[i].begin(), thread_costs[i].end());
            pairs_set.insert(pairs_set.end(), thread_pairs[i].begin(), thread_pairs[i].end());
        }

        // 各线程实际处理的轨迹数相加 = 本帧总共处理了多少条轨迹(含被碰撞淘汰的)。
        int total_processed = 0;
        for (int i = 0; i < num_threads; ++i) total_processed += thread_processed[i];
        // Logger::m_out << "multi_thread_2 " << Timer::Elapsed(d_t) << std::endl;

        if (costs.empty()) {
            RCLCPP_ERROR_STREAM_THROTTLE(robot->get_logger(), *robot->get_clock(), 1000,
                "No available trajectory after cleaning."
                << " state=" << static_cast<int>(robot->getRobotState())
                << " obstacles=" << robot->getDataMap().size()
                << " use_volume=" << !solve_volume_.empty()
                << " robot_radius=" << robot_radius_
                << " | 处理=" << total_processed
                << " 碰撞淘汰=" << reject_collision_.load(std::memory_order_relaxed)
                << " 太短淘汰=" << reject_too_short_.load(std::memory_order_relaxed));
            best_traj.second = false;
            return false;
        }

        normalize_costs(costs);

        const size_t max_elements = 10;
        const size_t top_n = std::min(max_elements, costs.size());

        std::vector<size_t> indices(costs.size());
        std::iota(indices.begin(), indices.end(), 0);

        std::sort(indices.begin(), indices.end(), [&](size_t i, size_t j) {
            return costs[i].total_cost_ < costs[j].total_cost_;
        });


        double J_min = costs[indices[0]].total_cost_;
        std::vector<double> costs_weights(top_n, 0.0);
        const double lambda = 1.0;
        double weight_sum = 0.0;


        double p = 2;
        for (size_t k = 0; k < top_n && k < indices.size(); ++k) {
            double normalized_step = static_cast<double>(top_n - k) / static_cast<double>(top_n);
            double ws = 0.0 + 1.0 * pow(normalized_step, p);

            size_t idx = indices[k];
            costs_weights[k] = std::exp(-(costs[idx].total_cost_ - J_min) / lambda) * ws;
            weight_sum += costs_weights[k];
        }

        if (weight_sum > 1e-6) {
            for (size_t k = 0; k < top_n && k < indices.size(); ++k) {
                costs_weights[k] /= weight_sum;
            }
        } else {
            RCLCPP_ERROR_STREAM_THROTTLE(robot->get_logger(), *robot->get_clock(), 1000,
                "Weight sum is zero. Check cost calculation.");
            return false;
        }

        delta_v_sum = 0.0;
        delta_w_sum = 0.0;
        delta_vy_sum = 0.0;

        for (size_t k = 0; k < top_n && k < indices.size(); ++k) {
            size_t idx = indices[k];
            for (size_t j = 0; j < static_cast<size_t>(nr_steps_); ++j) {
                delta_v_sum  += costs_weights[k] * pairs_set[idx][j].vx * timeInterval[j] / 2.0;
                delta_vy_sum += costs_weights[k] * pairs_set[idx][j].vy * timeInterval[j] / 2.0;
                delta_w_sum  += costs_weights[k] * pairs_set[idx][j].w  * timeInterval[j] / 2.0;
            }
        }

        auto velocity = robot->getVelocityLimits();
        delta_v_sum  = std::clamp(delta_v_sum,  velocity.min_linear,  velocity.max_linear);
        delta_vy_sum = std::clamp(delta_vy_sum, velocity.min_lateral, velocity.max_lateral);
        delta_w_sum  = std::clamp(delta_w_sum,  velocity.min_angular, velocity.max_angular);

        best_traj.first = generateTrajectory(state, state_odom, delta_v_sum, delta_vy_sum, delta_w_sum).first;

        best_traj.second = true;

        // 本帧：总共处理了多少条轨迹(各线程实际评估数之和)，其中生成了多少条有效轨迹(碰撞淘汰后剩下的)。
        RCLCPP_INFO_THROTTLE(
            robot->get_logger(), *robot->get_clock(), 1000,
            "处理轨迹=%d | 生成轨迹=%zu | 碰撞淘汰=%d 太短淘汰=%d",
            total_processed, costs.size(),
            reject_collision_.load(std::memory_order_relaxed),
            reject_too_short_.load(std::memory_order_relaxed));

        return true;

    }

    void DDP::process_segment(int thread_id, int start, int end, PoseState &state,
                              PoseState &state_odom, Window &dw,
                              std::vector<Control> &pairs,
                              std::vector<Cost> &thread_costs,
                              std::vector<std::vector<Control> > &thread_pairs,
                              int &processed_count) {
        (void)thread_id;
        (void)dw;

        processed_count = 0;   // 本线程实际处理(评估)的轨迹数

        // 本线程本地累加两类淘汰原因，循环结束后只对共享原子各做一次加法，避免 8 线程
        // 每条候选都抢同一个原子造成 cache 争用(那会拖慢整帧)。
        int collision_local = 0;
        int too_short_local = 0;


        static thread_local std::mt19937 gen(std::random_device{}());
        std::normal_distribution<double> linear_dist(0.0, linear_stddev);
        std::normal_distribution<double> lateral_dist(0.0, lateral_stddev);
        std::normal_distribution<double> angular_dist(0.0, angular_stddev);

        for (int i = start; i < end; ++i) {
            if (timeout_flag.load(std::memory_order_relaxed)) break;
            // 每 16 条查一次时钟(now() 很便宜，但不必每条都查)：超时预算则置位 flag，
            // 本线程与其它线程都会在下次循环顶部 break，用已算出的候选选解。
            if ((i & 0x0F) == 0 &&
                std::chrono::high_resolution_clock::now() >= solve_deadline_) {
                timeout_flag.store(true, std::memory_order_relaxed);
                break;
            }

            std::vector<Control> perturbations(nr_steps_);
            double dist = -1;
            std::vector<double> last_position;
            auto velocity = robot->getVelocityLimits();

            for (int j = 0; j < nr_steps_; ++j) {
                // 三维高斯扰动：在基准控制 (vx, vy, w) 附近采样，候选数不随维度爆炸。
                double sampled_vx = pairs[i].vx + linear_dist(gen);
                // use_vy_ 关闭(如 NORMAL 状态)：侧移恒 0，退化为差速(前向+转向)。
                double sampled_vy = use_vy_ ? (pairs[i].vy + lateral_dist(gen)) : 0.0;
                double sampled_w  = pairs[i].w  + angular_dist(gen);

                sampled_vx = std::clamp(sampled_vx, velocity.min_linear,  velocity.max_linear);
                sampled_vy = use_vy_ ? std::clamp(sampled_vy, velocity.min_lateral, velocity.max_lateral) : 0.0;
                sampled_w  = std::clamp(sampled_w,  velocity.min_angular, velocity.max_angular);

                perturbations[j] = {sampled_vx, sampled_vy, sampled_w};
            }

            std::pair<std::vector<PoseState>, std::vector<PoseState> > trajectories;
            trajectories = generateTrajectory(state, state_odom, perturbations);

            const Cost cost = evaluate_trajectory(trajectories, dist, last_position);
            ++processed_count;   // 每评估一条轨迹就计数(含被碰撞淘汰的)

            // 诊断：分别统计两类淘汰原因(碰撞 / 轨迹太短)，定位"没轨迹"到底卡在哪。
            if (cost.obs_cost_ == 1e6)  ++collision_local;
            if (cost.path_cost_ == 1e6) ++too_short_local;

            if (cost.obs_cost_ != 1e6 && cost.path_cost_ != 1e6 && cost.ori_cost_ != 1e6) {
                thread_pairs.emplace_back(perturbations);
                thread_costs.emplace_back(cost);
            }
        }

        // 收尾：本线程的两类淘汰计数一次性汇入共享原子(整帧每线程各 2 次原子加，无争用)。
        reject_collision_.fetch_add(collision_local, std::memory_order_relaxed);
        reject_too_short_.fetch_add(too_short_local, std::memory_order_relaxed);
    }

    std::pair<std::vector<PoseState>, std::vector<PoseState> >
    DDP::generateTrajectory(PoseState &state, PoseState &state_odom,
                            double angular_velocity) {
        std::pair<std::vector<PoseState>, std::vector<PoseState> > trajectory;
        trajectory.first.resize(nr_steps_);
        trajectory.second.resize(nr_steps_);
        PoseState state_ = state;
        PoseState state_odom_ = state_odom;

        n = 0.0;
        for (int i = 0; i < nr_steps_; ++i) {
            // 纯旋转候选：前/侧向速度≈0，仅施加偏航角速度。
            motion(state_, 0.0000001, 0.0, angular_velocity, timeInterval[i]);
            trajectory.first[i] = state_;
            motion(state_odom_, 0.0000001, 0.0, angular_velocity, timeInterval[i]);
            trajectory.second[i] = state_odom_;
        }

        return trajectory;
    }

    std::pair<std::vector<PoseState>, std::vector<PoseState> >
    DDP::generateTrajectory(PoseState &state, PoseState &state_odom,
                            std::vector<Control> &perturbations) {
        std::pair<std::vector<PoseState>, std::vector<PoseState> > trajectory;
        trajectory.first.resize(nr_steps_);
        trajectory.second.resize(nr_steps_);
        PoseState state_ = state;
        PoseState state_odom_ = state_odom;

        for (int i = 0; i < nr_steps_; i++) {
            motion(state_, perturbations[i].vx, perturbations[i].vy, perturbations[i].w, timeInterval[i]);
            trajectory.first[i] = state_;
            motion(state_odom_, perturbations[i].vx, perturbations[i].vy, perturbations[i].w, timeInterval[i]);
            trajectory.second[i] = state_odom_;
        }

        return trajectory;
    }

    std::pair<std::vector<PoseState>, std::vector<PoseState> >
    DDP::generateTrajectory(PoseState &state, PoseState &state_odom, const double vx,
                            const double vy, const double w) {
        std::pair<std::vector<PoseState>, std::vector<PoseState> > trajectory;
        trajectory.first.resize(nr_steps_);
        trajectory.second.resize(nr_steps_);
        PoseState state_ = state;
        PoseState state_odom_ = state_odom;

        for (int i = 0; i < nr_steps_; i++) {
            motion(state_, vx + 0.00001, vy, w, timeInterval[i]);
            trajectory.first[i] = state_;
            motion(state_odom_, vx + 0.00001, vy, w, timeInterval[i]);
            trajectory.second[i] = state_odom_;
        }

        return trajectory;
    }

    void DDP::normalize_costs(std::vector<DDP::Cost> &costs) {
        Cost min_cost(1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6), max_cost;

        for (const auto &cost: costs) {
            if (cost.obs_cost_ != 1e6 && cost.path_cost_ != 1e6 && cost.ori_cost_ != 1e6) {
                min_cost.obs_cost_ = std::min(min_cost.obs_cost_, cost.obs_cost_);
                max_cost.obs_cost_ = std::max(max_cost.obs_cost_, cost.obs_cost_);
                if (use_goal_cost_) {
                    min_cost.to_goal_cost_ = std::min(min_cost.to_goal_cost_, cost.to_goal_cost_);
                    max_cost.to_goal_cost_ = std::max(max_cost.to_goal_cost_, cost.to_goal_cost_);
                }
                if (use_ori_cost_) {
                    min_cost.ori_cost_ = std::min(min_cost.ori_cost_, cost.ori_cost_);
                    max_cost.ori_cost_ = std::max(max_cost.ori_cost_, cost.ori_cost_);
                }
                if (use_speed_cost_) {
                    min_cost.speed_cost_ = std::min(min_cost.speed_cost_, cost.speed_cost_);
                    max_cost.speed_cost_ = std::max(max_cost.speed_cost_, cost.speed_cost_);
                }
                if (use_path_cost_) {
                    min_cost.path_cost_ = std::min(min_cost.path_cost_, cost.path_cost_);
                    max_cost.path_cost_ = std::max(max_cost.path_cost_, cost.path_cost_);
                }
                if (use_angular_cost_) {
                    min_cost.aw_cost_ = std::min(min_cost.aw_cost_, cost.aw_cost_);
                    max_cost.aw_cost_ = std::max(max_cost.aw_cost_, cost.aw_cost_);
                }

                if (use_space_cost_) {
                    min_cost.space_cost_ = std::min(min_cost.space_cost_, cost.space_cost_);
                    max_cost.space_cost_ = std::max(max_cost.space_cost_, cost.space_cost_);
                }
            }
        }

        for (auto &cost: costs) {
            if (cost.obs_cost_ != 1e6 && cost.path_cost_ != 1e6 && cost.ori_cost_ != 1e6) {
                cost.obs_cost_ =
                        (cost.obs_cost_ - min_cost.obs_cost_) / (max_cost.obs_cost_ - min_cost.obs_cost_ + DBL_EPSILON);

                if (use_goal_cost_) {
                    cost.to_goal_cost_ = (cost.to_goal_cost_ - min_cost.to_goal_cost_) /
                                         (max_cost.to_goal_cost_ - min_cost.to_goal_cost_ + DBL_EPSILON);
                }

                if (use_ori_cost_)
                    cost.ori_cost_ =
                            (cost.ori_cost_ - min_cost.ori_cost_) /
                            (max_cost.ori_cost_ - min_cost.ori_cost_ + DBL_EPSILON);

                if (use_speed_cost_)
                    cost.speed_cost_ =
                            (cost.speed_cost_ - min_cost.speed_cost_) /
                            (max_cost.speed_cost_ - min_cost.speed_cost_ + DBL_EPSILON);

                if (use_path_cost_)
                    cost.path_cost_ =
                            (cost.path_cost_ - min_cost.path_cost_) /
                            (max_cost.path_cost_ - min_cost.path_cost_ + DBL_EPSILON);

                if (use_angular_cost_)
                    cost.aw_cost_ =
                            (cost.aw_cost_ - min_cost.aw_cost_) /
                            (max_cost.aw_cost_ - min_cost.aw_cost_ + DBL_EPSILON);

                if (use_space_cost_)
                    cost.space_cost_ =
                            (cost.space_cost_ - min_cost.space_cost_) /
                            (max_cost.space_cost_ - min_cost.space_cost_ + DBL_EPSILON);

                cost.to_goal_cost_ *= to_goal_cost_gain_;
                cost.obs_cost_ *= obs_cost_gain_;
                cost.speed_cost_ *= speed_cost_gain_;
                cost.path_cost_ *= path_cost_gain_;
                cost.ori_cost_ *= ori_cost_gain_;
                cost.aw_cost_ *= aw_cost_gain_;
                cost.space_cost_ *= space_cost_gain_;
                cost.calc_total_cost();
            }
        }
    }

    void DDP::motion(PoseState &state, const double vx_cmd, const double vy_cmd,
                     const double angular_velocity, double t) const {
        // 受加减速度限制逼近目标速度。vy 暂用与 vx 相同的加速度上下限(可后续单列)。
        double vx = updateVelocity(state.vx_, vx_cmd, maxAccelerSpeed, minAccelerSpeed, t);
        double vy = updateVelocity(state.vy_, vy_cmd, maxAccelerSpeed, minAccelerSpeed, t);
        double w  = updateVelocity(state.angular_velocity_, angular_velocity, maxAngularAccelerSpeed,
                                   minAngularAccelerSpeed, t);

        // 中点积分：用本步偏航中点角度做机体→世界旋转，降低狗快速转向时的离散误差。
        const double theta_mid = state.theta_ + 0.5 * w * t;
        const double c = std::cos(theta_mid), s = std::sin(theta_mid);

        // 机体系 (vx 前向, vy 左向) 旋转到世界系。
        state.x_ += (vx * c - vy * s) * t;
        state.y_ += (vx * s + vy * c) * t;
        state.theta_ += w * t;

        state.vx_ = vx;
        state.vy_ = vy;
        state.angular_velocity_ = w;

        state.theta_ = normalizeAngle(state.theta_);
    }

    double DDP::calc_speed_cost(const std::vector<PoseState> &traj) const {
        if (!use_speed_cost_)
            return 0.0;
        return std::abs(current_vel - traj[3].vx_);
    }

    double DDP::calc_to_goal_cost(const std::vector<PoseState> &traj) {
        if (use_goal_cost_ == false)
            return 0.0;
        double d = 0;
        for (int i = int(3 * traj.size() / 4); i < (int)traj.size() - 1; i++) {
            d += Algebra::PointDistance(2, &traj[i].pose()[0], &local_goal_odom[0]);
        }
        return d / int(1 * traj.size() / 4);
    }

    double DDP::calc_ori_cost(const std::vector<PoseState> &traj) {
        if (!use_ori_cost_)
            return 0.0;
        double theta = 0;
        for (int i = int(3 * traj.size() / 4); i < (int)traj.size() - 1; i++) {
            // |机体朝向 与 指向 local goal 方向 的夹角|，再按 ori_shape_p_ 整形：
            // p=1 线性；p>1 超线性(大偏离狠罚，RECOVER 用以强逼车头转正朝目标)。
            const double a = std::fabs(calculateTheta(traj[i], &local_goal_odom[0]));
            theta += (ori_shape_p_ == 1.0) ? a : std::pow(a, ori_shape_p_);
        }
        return theta / int(1 * traj.size() / 4);
    }

    double DDP::calc_path_cost(const std::vector<PoseState> &traj) const {
        // 先用轨迹自身长度判废：太短(原地不动)的候选直接 1e6 丢弃。
        double traj_len = 0;
        for (size_t i = 0; i < traj.size() - 2; i++)
            traj_len += Algebra::PointDistance(2, &traj[i].pose()[0], &traj[i + 1].pose()[0]);

        if (traj_len <= min_traj_length_)
            return 1e6;

        if (!use_path_cost_)
            return 0.0;

        std::vector<std::vector<double> > global_path = robot->global_paths_odom;
        if (global_path.empty()) {
            return 0.0;
        }

        // 纯 cross-track：从 0 累加每个轨迹点到全局路径的最近距离(不再混入轨迹长度)。
        double d = 0;
        for (const auto &state: traj) {
            double min_distance = std::numeric_limits<double>::max();
            for (const auto &point: global_path) {
                if (point.size() < 2) continue; // Ensure the point has at least x and y coordinates
                double dx = state.x_ - point[0];
                double dy = state.y_ - point[1];
                double pt_dist = std::sqrt(dx * dx + dy * dy);
                if (pt_dist < min_distance) {
                    min_distance = pt_dist;
                }
            }
            d += min_distance;
        }

        return d / (double) traj.size();
    }


    double DDP::calc_obs_cost(const std::vector<PoseState> &traj, double &t) {
        // 障碍来自本帧预处理好的扁平数组(ddp_planning 已取一次 getDataMap + 可达预筛 +
        // 扁平化)，不再每条候选轨迹深拷贝 vector<vector<double>>，且遍历 cache 友好。
        const std::vector<float> &obs_x = solve_obs_x_;
        const std::vector<float> &obs_y = solve_obs_y_;
        const size_t n_obs = obs_x.size();


        double cost = 0.0;
        double space_cost = 0.0;


        const double angle_threshold = M_PI / 4;
        // 步数与"后 1/4 段"起点只算一次，不再在内层循环重复计算。
        const size_t steps = traj.size();
        const size_t rear_start = static_cast<size_t>(3 * steps / 4);

        // ===== 轨迹级 AABB 预筛 =====
        // 800 候选轨迹 / ~500 障碍 / 19 段 → 内层循环 ~7.6M 次。绝大多数障碍离整条
        // 轨迹都很远，对【任何】步都不可能产生非零代价。先扫一遍 traj 算出包围盒，
        // 向外扩 (robot 外接圆 + influence + 半个体盒)，把不在此 box 内的障碍一次
        // 性过滤掉；内层循环长度通常从 ~500 砍到几十，整体 ~10× 加速。
        double xmin = std::numeric_limits<double>::infinity();
        double ymin = xmin;
        double xmax = -xmin;
        double ymax = -ymin;
        for (const auto &p : traj) {
            if (p.x_ < xmin) xmin = p.x_;
            if (p.x_ > xmax) xmax = p.x_;
            if (p.y_ < ymin) ymin = p.y_;
            if (p.y_ > ymax) ymax = p.y_;
        }
        // 扩边：远场粗筛阈值 = 真实模型外接圆半径 + 影响半径 + robot_radius_(状态安全裕度)，
        // 确保所有可能产生非零代价的障碍都被纳入 near[]。
        const double pad = solve_volume_circ_r_ + obs_influence_dist_ + robot_radius_;
        const double x_lo = xmin - pad, x_hi = xmax + pad;
        const double y_lo = ymin - pad, y_hi = ymax + pad;

        // near[] 只存通过 box 的障碍坐标(连续存储, cache 友好), 内层循环直接读。
        std::vector<std::pair<float, float>> near;
        near.reserve(n_obs);
        for (size_t k = 0; k < n_obs; ++k) {
            const float ox = obs_x[k], oy = obs_y[k];
            if (ox < x_lo || ox > x_hi || oy < y_lo || oy > y_hi) continue;
            near.emplace_back(ox, oy);
        }

        // 距离 -> 障碍代价的【归一化势场】映射，按模式参数化(见 DDP.hpp / Parameters.cpp)：
        // 在 [lethal, influence] 上连续地从峰值(如300)降到0，近处陡(1/md)，两端无跳变。
        const double inv_infl   = 1.0 / obs_influence_dist_;
        const double inv_lethal = 1.0 / obs_lethal_dist_;
        const double q_max = inv_lethal - inv_infl;   // 归一化分母(>0)
        auto obstacleCost = [this, inv_infl, q_max](double md) -> double {
            if (md <= obs_lethal_dist_)    return obs_lethal_cost_;   // 峰值
            if (md >= obs_influence_dist_) return 0.0;                // 影响带外
            const double ratio = (1.0 / md - inv_infl) / q_max;       // ∈[0,1]
            return obs_lethal_cost_ * std::pow(ratio, obs_shape_p_);
        };

        // 障碍点到中心线线段 a->b 的欧氏距离(无三角函数，便宜)。用作 volume 分支
        // 的【线段粗筛】：把"障碍到 traj[i] 单点"升级为"障碍到整条线段"，远场判定
        // 即覆盖 traj[i]→traj[i+1] 之间，避免长步长时中间段被漏看。
        auto segClearance = [](const PoseState &a, const PoseState &b,
                               double ox, double oy) -> double {
            const double abx = b.x_ - a.x_;
            const double aby = b.y_ - a.y_;
            const double apx = ox - a.x_;
            const double apy = oy - a.y_;
            const double L2 = abx * abx + aby * aby;
            double tproj = (L2 > 1e-12) ? (apx * abx + apy * aby) / L2 : 0.0;
            if (tproj < 0.0) tproj = 0.0; else if (tproj > 1.0) tproj = 1.0;
            const double cx = a.x_ + tproj * abx;
            const double cy = a.y_ + tproj * aby;
            return std::hypot(ox - cx, oy - cy);
        };

        for (size_t i = 0; i + 1 < steps; ++i) {
            double min_dist = FLT_MAX;
            double min_front_dist = 3.0;

            // 是否进入需要统计 space_cost 的"前向锥"判定；与障碍无关，每步算一次。
            const bool rear = use_space_cost_ && (i > rear_start);
            double traj_angle = 0.0;
            if (rear) {
                // traj_angle 只依赖步 i，不依赖障碍：提到障碍循环外，避免每个障碍重复 atan2。
                traj_angle = std::atan2(traj[i + 1].y_ - traj[i].y_,
                                        traj[i + 1].x_ - traj[i].x_);
            }

            for (const auto &np : near) {
                const double ox = np.first;
                const double oy = np.second;

                // 线段粗筛：障碍到中心线线段 traj[i]→traj[i+1] 的距离(便宜)，减外接圆
                // 半径 + robot_radius_(状态安全裕度) 得真实边缘距离的【保守下界】，已覆盖整条线段。
                const double seg_d = segClearance(traj[i], traj[i + 1], ox, oy);
                const double circ_clear = seg_d - solve_volume_circ_r_ - robot_radius_;
                double dist;
                if (circ_clear >= obs_influence_dist_) {
                    // 远场:连这个下界都在影响半径外,代价必为0,直接用,免精算。
                    dist = circ_clear;
                } else {
                    // 近场:沿整段运动自适应子步精算(连续覆盖线段、含旋转)，取最小有符号
                    // 距离，再减 robot_radius_ 裕度。子步只在少数近场障碍上发生,开销可控。
                    dist = solve_volume_.distanceToSegment(
                        traj[i].x_,     traj[i].y_,     traj[i].theta_,
                        traj[i + 1].x_, traj[i + 1].y_, traj[i + 1].theta_,
                        ox, oy) - robot_radius_;
                }

                if (dist < DBL_EPSILON) {
                    return 1e6;
                }

                if (dist < min_dist) min_dist = dist;

                if (rear) {
                    const double dx = ox - traj[i].x_;  // STATE_X = 0
                    const double dy = oy - traj[i].y_;  // STATE_Y = 1
                    double angle_diff = std::fabs(traj_angle - std::atan2(dy, dx));
                    if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
                    if (angle_diff < angle_threshold && dist < min_front_dist) {
                        min_front_dist = dist;
                    }
                }
            }

            cost += obstacleCost(min_dist);

            if (rear) {
                space_cost += std::max(obs_range_ - min_front_dist, 0.0);
            }
        }

        t = space_cost;

        return cost;
    }


    DDP::Cost DDP::evaluate_trajectory(
        std::pair<std::vector<PoseState>, std::vector<PoseState> > &trajectory,
        double &dist, std::vector<double> &last_position) {
        (void)dist;
        (void)last_position;
        Cost cost;
        double t = 0.0;
        cost.to_goal_cost_ = calc_to_goal_cost(trajectory.first);
        cost.obs_cost_ = calc_obs_cost(trajectory.first, t);
        cost.space_cost_ = t;
        cost.speed_cost_ = calc_speed_cost(trajectory.first);
        cost.path_cost_ = calc_path_cost(trajectory.first);
        cost.ori_cost_ = calc_ori_cost(trajectory.first);
        cost.aw_cost_ = calc_angular_velocity(trajectory.first);

        cost.calc_total_cost();
        return cost;
    }


}
