// Go2_callbacks.cpp — ROS2 callback implementations
#include "Go2_callbacks.hpp"
#include "Go2.hpp"
#include <cmath>

Go2Callbacks::Go2Callbacks(Robot_config* robot)
    : robot_(robot) {
}

void Go2Callbacks::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!robot_) return;

    double cur_x, cur_y;
    {
        std::lock_guard<std::mutex> lock(robot_->robot_state_mutex_);
        robot_->robot_state.x_ = msg->pose.pose.position.x;
        robot_->robot_state.y_ = msg->pose.pose.position.y;

        // Extract yaw from quaternion
        double siny_cosp = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                                  msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
        double cosy_cosp = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                                        msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
        robot_->robot_state.theta_ = std::atan2(siny_cosp, cosy_cosp);

        robot_->robot_state.velocity_ = msg->twist.twist.linear.x;
        robot_->robot_state.angular_velocity_ = msg->twist.twist.angular.z;
        robot_->robot_state.valid_ = true;

        cur_x = robot_->robot_state.x_;
        cur_y = robot_->robot_state.y_;
    }  // 先释放 robot_state_mutex_，再去拿 path_goal_mutex_，避免与
       // processValidGlobalPath(先 path_goal_mutex_ 再 robot_state_mutex_) 形成反向加锁死锁。

    // 到达判定：离 global goal 1m 内即视为到达，停止规划，防止在目标附近一直打转。
    checkGoalReached(cur_x, cur_y);
}

void Go2Callbacks::checkGoalReached(double cur_x, double cur_y) {
    std::lock_guard<std::mutex> lock(robot_->path_goal_mutex_);

    // 还没有全局目标就不判定
    if (!robot_->global_goal_received || robot_->global_goal.size() < 2) return;

    const double dx = robot_->global_goal[0] - cur_x;
    const double dy = robot_->global_goal[1] - cur_y;
    const double dist = std::sqrt(dx * dx + dy * dy);

    if (dist <= robot_->goal_reached_threshold) {
        if (!robot_->goal_reached) {
            RCLCPP_INFO(robot_->get_logger(),
                "到达 global goal (距离 %.2fm <= %.2fm)，停止规划，等待新目标。",
                dist, robot_->goal_reached_threshold);
        }
        robot_->goal_reached = true;
        // 让 setup() 不再放行 -> DDP 不发 /cmd_vel -> 桥看门狗 0.5s 后停狗。
        robot_->local_goal_received = false;
    }
}

void Go2Callbacks::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!robot_) return;

    std::lock_guard<std::mutex> lock(robot_->laser_data_mutex_);
    robot_->laserData.clear();
    robot_->laserDataDistance.clear();

    const auto& pose = robot_->getPoseState();

    // 过滤掉离机器人太近的激光点：Go2 的 3D lidar 转 2D 后会把狗自己的腿/身体当作障碍，
    // 这些点（实测有 ~170 个在 0.5m 内、其中 3 个 < 0.3m）会让 DDP 把每条候选轨迹都判为撞障，
    // 导致 "No available trajectory after cleaning"。SELF_FILTER 半径取 0.30m，对应狗自身
    // 占地的外包圆(机身 0.72×0.36 → 半对角 ≈0.40m，留点裕度过滤稍紧到 0.30m，避免把真实近障也滤掉)。
    constexpr float SELF_FILTER_RANGE = 0.30f;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float range = msg->ranges[i];
        if (range < msg->range_min || range > msg->range_max || std::isnan(range) || std::isinf(range)) {
            continue;
        }
        if (range < SELF_FILTER_RANGE) {
            continue;  // 视为机器人自身回波
        }

        float angle = msg->angle_min + i * msg->angle_increment;
        float global_angle = angle + static_cast<float>(pose.theta_);

        float x = static_cast<float>(pose.x_) + range * std::cos(global_angle);
        float y = static_cast<float>(pose.y_) + range * std::sin(global_angle);

        robot_->laserData.emplace_back(x, y);
        robot_->laserDataDistance.push_back(static_cast<double>(range));
    }
}

void Go2Callbacks::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (!robot_) return;

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
    if (!robot_) return;

    if (msg->poses.empty()) {
        handleEmptyGlobalPath();
        return;
    }

    processValidGlobalPath(msg);
}

bool Go2Callbacks::handleEmptyGlobalPath() {
    robot_->setRobotState(Robot_config::BRAKE_PLANNING);
    return false;
}

void Go2Callbacks::processValidGlobalPath(const nav_msgs::msg::Path::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(robot_->path_goal_mutex_);

    robot_->global_paths.clear();
    robot_->global_paths_odom.clear();

    for (const auto& pose : msg->poses) {
        std::vector<double> point = {pose.pose.position.x, pose.pose.position.y};
        robot_->global_paths.push_back(point);
        robot_->global_paths_odom.push_back(point);
    }

    // Set local goal from path
    if (!msg->poses.empty()) {
        // 已到达 global goal：保持停止，不再用 /plan 重新激活 local_goal_received，
        // 否则 2Hz 的 /plan 会立刻把刚到达时清掉的标志重新置位，狗又动起来打转。
        if (robot_->goal_reached) {
            robot_->local_goal_received = false;
        } else {
            double threshold = computeLookaheadThreshold();
            const auto& current_pose = robot_->getPoseState();

            bool found = false;
            for (const auto& pose : msg->poses) {
                double dx = pose.pose.position.x - current_pose.x_;
                double dy = pose.pose.position.y - current_pose.y_;
                double dist = std::sqrt(dx * dx + dy * dy);

                if (dist >= threshold) {
                    robot_->local_goal = {pose.pose.position.x, pose.pose.position.y};
                    // /plan 已是 odom 帧，local_goal 本身即 odom 坐标，保持 odom 版一致
                    robot_->local_goal_odom = robot_->local_goal;
                    robot_->local_goal_received = true;
                    found = true;
                    break;
                }
            }

            // If no point far enough, use last point
            if (!found) {
                const auto& last = msg->poses.back();
                robot_->local_goal = {last.pose.position.x, last.pose.position.y};
                robot_->local_goal_odom = robot_->local_goal;
                robot_->local_goal_received = true;
            }

            // 收到有效全局路径且里程计已就绪时，离开 INITIALIZING 进入正常规划
            // （否则状态机会永远卡在 INITIALIZING，setup() 永远回不了 true）
            if (robot_->getRobotState() == Robot_config::INITIALIZING &&
                robot_->getPoseState().valid_) {
                robot_->setRobotState(Robot_config::NORMAL_PLANNING);
            }
        }
    }

    // 每收到一条有效 /plan 就发布目标可视化(odom 帧)：local_goal 绿、global_goal 红。
    // 此处已持有 path_goal_mutex_，view_Goal 内部只发 Marker、不加锁，安全。
    robot_->view_Goal(robot_->local_goal_odom, robot_->global_goal_odom);
}

double Go2Callbacks::computeLookaheadThreshold() const {
    return robot_->local_goal_distance;  // Use local_goal_distance parameter
}

void Go2Callbacks::timeIntervalCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (!robot_) return;

    std::lock_guard<std::mutex> lock(robot_->timeInterval_mutex_);
    robot_->timeInterval.clear();
    robot_->timeInterval.assign(msg->data.begin(), msg->data.end());
}

void Go2Callbacks::paramsCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (!robot_) return;
    robot_->param_received = true;
    // Parse parameters from string if needed
    // Format could be JSON or key=value pairs
    (void)msg;  // TODO: Implement parameter parsing if needed
}

void Go2Callbacks::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!robot_) return;

    std::lock_guard<std::mutex> lock(robot_->path_goal_mutex_);
    robot_->global_goal.clear();
    robot_->global_goal.push_back(msg->pose.position.x);
    robot_->global_goal.push_back(msg->pose.position.y);

    // Extract yaw from quaternion
    double siny_cosp = 2.0 * (msg->pose.orientation.w * msg->pose.orientation.z +
                              msg->pose.orientation.x * msg->pose.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (msg->pose.orientation.y * msg->pose.orientation.y +
                                    msg->pose.orientation.z * msg->pose.orientation.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    robot_->global_goal.push_back(yaw);
    // /goal_pose 已是 odom 帧，global_goal 本身即 odom 坐标，保持 odom 版一致，
    // 否则 view_Goal 里 goal1.size()<2 会导致红色全局目标 Marker 永远不发布。
    robot_->global_goal_odom = robot_->global_goal;
    robot_->global_goal_received = true;
    // 收到新目标 -> 复位到达标志，重新开始规划。
    robot_->goal_reached = false;
}

void Go2Callbacks::velocityCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!robot_) return;

    std::lock_guard<std::mutex> lock(robot_->robot_state_mutex_);
    robot_->robot_state.velocity_ = msg->twist.twist.linear.x;
    robot_->robot_state.angular_velocity_ = msg->twist.twist.angular.z;
}
