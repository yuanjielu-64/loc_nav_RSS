// Go2_callbacks.cpp — ROS2 callback implementations
#include "Go2_callbacks.hpp"
#include "Go2.hpp"
#include "Utility.hpp"   // transform_lg(world→base_link)（goalCallback 仍用）
#include "../globalPlanners/GlobalPathHandler.hpp"   // /plan 处理实体(valid/empty)
#include "../perception/LaserScanProcessor.hpp"      // 一帧激光的总处理(odom 点 + 8 方向余量)
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_set>

Go2Callbacks::Go2Callbacks(Robot_config* robot)
    : robot_(robot) {
}

void Go2Callbacks::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const auto& p  = msg->pose.pose.position;
    const auto& q  = msg->pose.pose.orientation;
    const auto& tw = msg->twist.twist;

    // 四元数 -> yaw（绕 z 的旋转）
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    {
        std::lock_guard<std::mutex> lock(robot_->robot_state_mutex_);
        robot_->robot_state.x_ = p.x;
        robot_->robot_state.y_ = p.y;
        robot_->robot_state.theta_ = yaw;
        robot_->robot_state.vx_ = tw.linear.x;
        robot_->robot_state.vy_ = tw.linear.y;
        robot_->robot_state.angular_velocity_ = tw.angular.z;
        robot_->robot_state.valid_ = true;
    }
}

void Go2Callbacks::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // 整段激光处理(极坐标→odom 点 + 8 方向边缘余量)已抽到 perception::processLaserScan，
    // 回调只负责：取位姿/轮廓模型 → 调用 → 把结果加锁写回 Robot_config。
    // 用【激光那一帧时间戳】对应的位姿(TF odom→base_link@stamp)，而不是最新位姿——
    // 否则运动/转弯时会用新位姿变换旧激光，导致激光点整体偏移(致命)。
    const auto pose = robot_->getPoseStateAt(msg->header.stamp);
    const auto dynamic_models = robot_->checkVolumeReady()
        ? robot_->getDynamicVolumes() : std::vector<go2::Footprint>{};

    auto scan_result = perception::processLaserScan(
        *msg, pose.x_, pose.y_, pose.theta_,
        dynamic_models, robot_->getStaticVolumes());

    // 把 perception 处理后的激光点(odom 系)发到 /perception/laser_points，供 RViz 核对。
    // 在 move 入库前用本地结果发布，避免读已被 move 走/竞争的 laserData_odom。
    robot_->viewLaserPoints(scan_result.points_odom);

    {
        std::lock_guard<std::mutex> lock(robot_->laser_data_mutex_);
        robot_->laserData_odom    = std::move(scan_result.points_odom);
        robot_->laserDataDistance = std::move(scan_result.ranges);
    }
    {
        std::lock_guard<std::mutex> olock(robot_->obstacle_mutex_);
        robot_->direction_clearance_ = scan_result.direction_clearance;
    }
}

void Go2Callbacks::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // 当前规划只用激光，costmapData 无人消费；不开关就别白遍历整张栅格。
    // 将来启用 BACK 倒车用 costmap 时把 robot_->use_costmap_ 置 true 即可。
    if (!robot_->use_costmap_) return;

    // Costmap callback - store obstacle cells
    robot_->costmapData.clear();

    double resolution = msg->info.resolution;
    double origin_x = msg->info.origin.position.x;
    double origin_y = msg->info.origin.position.y;

    for (unsigned int y = 0; y < msg->info.height; ++y) {
        for (unsigned int x = 0; x < msg->info.width; ++x) {
            int8_t value = msg->data[y * msg->info.width + x];
            // Consider cells with cost > 50 as obstacles
            if (value > 50) {
                double world_x = origin_x + x * resolution;
                double world_y = origin_y + y * resolution;
                robot_->costmapData.push_back({world_x, world_y});
            }
        }
    }
}

void Go2Callbacks::globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    // /plan 的处理实体已抽到 globalPlanners/GlobalPathHandler，这里只按是否空路径分流。
    if (msg->poses.empty())
        GlobalPathHandler::processEmptyGlobalPath(robot_);
    else
        GlobalPathHandler::processValidGlobalPath(robot_, msg);
}


void Go2Callbacks::timeIntervalCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(robot_->timeInterval_mutex_);
    robot_->timeInterval.clear();
    robot_->timeInterval.assign(msg->data.begin(), msg->data.end());
}

void Go2Callbacks::paramsCallback(const std_msgs::msg::String::SharedPtr msg) {
    robot_->param_received = true;
    (void)msg;  // TODO: Implement parameter parsing if needed
}

void Go2Callbacks::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(robot_->path_goal_mutex_);

    const double gx = msg->pose.position.x;
    const double gy = msg->pose.position.y;

    // Extract yaw from quaternion
    double siny_cosp = 2.0 * (msg->pose.orientation.w * msg->pose.orientation.z +
                              msg->pose.orientation.x * msg->pose.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (msg->pose.orientation.y * msg->pose.orientation.y +
                                    msg->pose.orientation.z * msg->pose.orientation.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    const auto rp = robot_->getPoseState();
    auto body = transform_lg(gx, gy, rp.x_, rp.y_, rp.theta_);
    const double yaw_body = std::atan2(std::sin(yaw - rp.theta_), std::cos(yaw - rp.theta_));

    robot_->global_goal = {body[0], body[1], yaw_body};
    robot_->global_goal_odom = {gx, gy, yaw};
    robot_->global_goal_received = true;
    robot_->global_goal_reached = false;
    robot_->local_goal_received = false;

    robot_->setRobotState(Robot_config::NORMAL);
}

void Go2Callbacks::velocityCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(robot_->robot_state_mutex_);
        robot_->robot_state.vx_ = msg->twist.twist.linear.x;
        robot_->robot_state.vy_ = msg->twist.twist.linear.y;   // 全向：机体侧向速度
        robot_->robot_state.angular_velocity_ = msg->twist.twist.angular.z;
    }
    // 速度状态机(NORMAL/CAUTIOUS/BRAKE 自动切换)：实现见 Go2_prepare.cpp。
    robot_->updateSpeedStateMachine(std::fabs(msg->twist.twist.linear.x));
}

void Go2Callbacks::robotVolumeCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    auto parsed = go2::Footprint::parseFootprints(*msg);
    if (parsed.empty()) return;   // 空/无效消息：保留上一帧，不覆盖
    {
        std::lock_guard<std::mutex> lock(robot_->volumes_mutex_);
        robot_->models_dynamic_ = std::move(parsed);
    }
    robot_->volume_received = true;
}

