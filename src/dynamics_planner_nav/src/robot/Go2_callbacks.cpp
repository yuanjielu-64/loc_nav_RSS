// Go2_callbacks.cpp — ROS2 callback implementations
#include "Go2_callbacks.hpp"
#include "Go2.hpp"
#include <cmath>

Go2Callbacks::Go2Callbacks(Robot_config* robot)
    : robot_(robot) {
}

void Go2Callbacks::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!robot_) return;

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
}

void Go2Callbacks::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!robot_) return;

    std::lock_guard<std::mutex> lock(robot_->laser_data_mutex_);
    robot_->laserData.clear();
    robot_->laserDataDistance.clear();

    const auto& pose = robot_->getPoseState();

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float range = msg->ranges[i];
        if (range < msg->range_min || range > msg->range_max || std::isnan(range) || std::isinf(range)) {
            continue;
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
        double threshold = computeLookaheadThreshold();
        const auto& current_pose = robot_->getPoseState();

        bool found = false;
        for (const auto& pose : msg->poses) {
            double dx = pose.pose.position.x - current_pose.x_;
            double dy = pose.pose.position.y - current_pose.y_;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist >= threshold) {
                robot_->local_goal = {pose.pose.position.x, pose.pose.position.y};
                robot_->local_goal_received = true;
                found = true;
                break;
            }
        }

        // If no point far enough, use last point
        if (!found) {
            const auto& last = msg->poses.back();
            robot_->local_goal = {last.pose.position.x, last.pose.position.y};
            robot_->local_goal_received = true;
        }
    }
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
    robot_->global_goal_received = true;
}

void Go2Callbacks::velocityCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!robot_) return;

    std::lock_guard<std::mutex> lock(robot_->robot_state_mutex_);
    robot_->robot_state.velocity_ = msg->twist.twist.linear.x;
    robot_->robot_state.angular_velocity_ = msg->twist.twist.angular.z;
}
