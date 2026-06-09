// Go2.cpp — Robot state manager and ROS2 interface
#include "Go2.hpp"
#include "Go2_callbacks.hpp"
#include <cmath>
#include <nav_msgs/srv/get_plan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "std_srvs/srv/empty.hpp"
#include "Utility.hpp"

bool Robot_config::setup() {
    if (!checkGazeboPaused() && getRobotState() != INITIALIZING && getPoseState().valid_ && getMapData() && local_goal_received) {
        return true;
    }
    return false;
}

bool Robot_config::checkGazeboPaused() const {
    std_msgs::msg::String state_msg;

    // In ROS2, check parameter differently
    bool is_paused = false;
    // Note: In ROS2, you would typically use a parameter or service
    // For now, assume not paused
    if (is_paused) {
        state_msg.data = "PAUSED";
        robot_state_pub_->publish(state_msg);
        return true;
    }

    publishRobotState();
    return false;
}

bool Robot_config::getMapData() {
    const RobotState state = getRobotState();
    MapSource mapSource;
    map.clear();

    if (state == BACKWARD)
        mapSource = Robot_config::ONLY_COSTMAP_RECEIVED;
    else
        mapSource = Robot_config::ONLY_LASER_RECEIVED;

    const auto &primaryData = (mapSource == ONLY_COSTMAP_RECEIVED) ? costmapData : getLaserData();
    const auto &fallbackData = (mapSource == ONLY_COSTMAP_RECEIVED) ? getLaserData() : costmapData;

    if (!primaryData.empty()) {
        map = primaryData;
        currentMap = mapSource;
    } else if (!fallbackData.empty()) {
        map = fallbackData;
        currentMap = (mapSource == ONLY_COSTMAP_RECEIVED) ? ONLY_LASER_RECEIVED : ONLY_COSTMAP_RECEIVED;
    } else {
        currentMap = NO_ANY_RECEIVED;
        setRobotState(NO_MAP_PLANNING);
        return true;
    }

    return !map.empty();
}

Robot_config::Footprint Robot_config::getFootprint() const {
    const RobotState state = getRobotState();

    if (state == BACKWARD) {
        return {POINT_MASS_LENGTH, POINT_MASS_WIDTH};
    }

    if (currentMap != ONLY_LASER_RECEIVED) {
        return {POINT_MASS_LENGTH, POINT_MASS_WIDTH};
    }

    return {ROBOT_LENGTH, ROBOT_WIDTH};
}

Robot_config::VelocityLimits Robot_config::getVelocityLimits() const {
    const RobotState state = getRobotState();

    if (state == BACKWARD) {
        return {-2.0, 0.0, -2.0, 2.0};
    }

    if (state == FORWARD) {
        return {0.0, 2.0, -2.0, 2.0};
    }

    return {0.0, max_vel_x, -max_vel_theta, max_vel_theta};
}

std::vector<std::vector<double>> Robot_config::getLaserData() {
    std::vector<std::vector<double>> out;
    out.reserve(laserData.size());
    for (const auto &p: laserData) {
        out.push_back({static_cast<double>(p.x()), static_cast<double>(p.y())});
    }
    return out;
}

//==============================================================================
// Constructor: Initialize state and setup ROS2 communication
//==============================================================================
Robot_config::Robot_config()
    : Node("dynamics_planner_nav"),
      algorithm(DWA),
      currentState(INITIALIZING),
      currentMap(ONLY_LASER_RECEIVED),
      local_goal_received(false),
      global_goal_received(false),
      param_received(false),
      canBeSolved(true),
      rotating_angle(0.0),
      dt(0.05),
      latter_obs(INFINITY),
      front_obs(INFINITY),
      recover_times(0) {

    global_goal.reserve(2);
    local_goal.reserve(2);
    local_goal_odom.reserve(2);

    // Initialize state
    local_goal = {0.0, 0.0};
    robot_state = PoseState(0.0, 0.0, 0.0, 0.0, 0.0, false);
    actions = {{0.0, 0.0}};

    // Initialize time
    normal_to_low_time = this->now();
    low_to_normal_time = this->now();
    low_to_brake_time = this->now();

    // ---- Create Callback Handler ----
    callbacks_ = std::make_shared<Go2Callbacks>(this);

    // ---- Create Async Task Executor ----
    async_executor_ = std::make_shared<AsyncTaskExecutor>(num_threads);

    // ---- ROS2 Subscriptions ----
    robot_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/utlidar/robot_odom", 10,
        std::bind(&Go2Callbacks::odometryCallback, callbacks_.get(), std::placeholders::_1));

    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/front/scan", 10,
        std::bind(&Go2Callbacks::laserScanCallback, callbacks_.get(), std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        std::bind(&Go2Callbacks::goalCallback, callbacks_.get(), std::placeholders::_1));

    costmap_update_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/local_costmap/costmap", 10,
        std::bind(&Go2Callbacks::costmapCallback, callbacks_.get(), std::placeholders::_1));

    velocity_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/utlidar/robot_odom", 10,
        std::bind(&Go2Callbacks::velocityCallback, callbacks_.get(), std::placeholders::_1));

    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", 10,
        std::bind(&Go2Callbacks::globalPathCallback, callbacks_.get(), std::placeholders::_1));

    array_dt_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/dy_dt", 1,
        std::bind(&Go2Callbacks::timeIntervalCallback, callbacks_.get(), std::placeholders::_1));

    params_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/params", 1,
        std::bind(&Go2Callbacks::paramsCallback, callbacks_.get(), std::placeholders::_1));

    // ---- ROS2 Publishers ----
    trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);
    global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);
    smoothed_global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("smoothed_global_path", 10);
    local_goal_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("local_goal", 1);
    global_goal_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("global_goal", 1);
    tuning_params_pub_ = this->create_publisher<std_msgs::msg::String>("/tuning_params", 1);
    obstacles_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/teb_obstacles", 1);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    robot_state_pub_ = this->create_publisher<std_msgs::msg::String>("/robot_mode", 1);

    // ---- ROS2 Service Clients ----
    global_path_clt_ = this->create_client<nav_msgs::srv::GetPlan>("/compute_path_to_pose");
    clear_costmaps_clt_ = this->create_client<std_srvs::srv::Empty>("/clear_costmaps");

    RCLCPP_INFO(this->get_logger(), "Robot_config initialized successfully");
    RCLCPP_INFO(this->get_logger(), "All planners will use %d parallel threads", num_threads);

    // Initialize tuning snapshot from current defaults
    tuning_params_ = getTuningParams();
}

double Robot_config::calculateTheta(const PoseState &state, const std::vector<double> &y) {
    const double deltaX = y[0] - state.x_;
    const double deltaY = y[1] - state.y_;
    const double theta = std::atan2(deltaY, deltaX);
    const double normalizedTheta = normalize_angle(state.theta_);
    return std::fabs(normalize_angle(theta - normalizedTheta));
}

Robot_config::TuningParams Robot_config::getTuningParams() const {
    TuningParams params{};
    params.max_vel_x = max_vel_x;
    params.max_vel_y = max_vel_y;
    params.max_vel_theta = max_vel_theta;
    params.vx_sample = static_cast<int>(vx_sample);
    params.vTheta_samples = static_cast<int>(vTheta_samples);
    params.path_distance_bias = path_distance_bias;
    params.goal_distance_bias = goal_distance_bias;
    params.nr_pairs_ = static_cast<int>(nr_pairs_);
    params.nr_steps_ = static_cast<int>(nr_steps_);
    params.linear_stddev = linear_stddev;
    params.angular_stddev = angular_stddev;
    params.lambda = lambda;
    params.local_goal_distance = local_goal_distance;
    params.distance = distance;
    params.robot_radius_ = robot_radius_;
    params.dt = dt;
    return params;
}

void Robot_config::setTuningParams(const TuningParams &tp) {
    max_vel_x = tp.max_vel_x;
    max_vel_y = tp.max_vel_y;
    max_vel_theta = tp.max_vel_theta;
    vx_sample = tp.vx_sample;
    vTheta_samples = tp.vTheta_samples;
    path_distance_bias = tp.path_distance_bias;
    goal_distance_bias = tp.goal_distance_bias;
    nr_pairs_ = tp.nr_pairs_;
    nr_steps_ = tp.nr_steps_;
    linear_stddev = tp.linear_stddev;
    angular_stddev = tp.angular_stddev;
    lambda = tp.lambda;
    local_goal_distance = tp.local_goal_distance;
    distance = tp.distance;
    robot_radius_ = tp.robot_radius_;
    dt = tp.dt;
    tuning_params_ = tp;
}

void Robot_config::update_angular_velocity() {
    if (getAlgorithm() == DWA || getAlgorithm() == DWA_DDP) {
        if (getRobotState() == NORMAL_PLANNING)
            max_vel_theta = 2;
        else if (getRobotState() == LOW_SPEED_PLANNING)
            max_vel_theta = 1;
    } else {
        if (getRobotState() == NORMAL_PLANNING) {
            if (std::abs(getPoseState().angular_velocity_) <= 1 &&
                std::abs(getPoseState().velocity_) <= 1 * max_vel_x / 3)
                max_vel_theta = 2;
            else if ((std::abs(getPoseState().angular_velocity_) <= 2 &&
                      std::abs(getPoseState().angular_velocity_) > 1 * max_vel_x / 3) ||
                     (std::abs(getPoseState().velocity_) > 1 &&
                      std::abs(getPoseState().velocity_) <= 2 * max_vel_x / 3))
                max_vel_theta = 1.5;
            else
                max_vel_theta = 1.0;
        } else if (getRobotState() == LOW_SPEED_PLANNING) {
            if (std::abs(getPoseState().angular_velocity_) <= 1 &&
                std::abs(getPoseState().velocity_) <= 1 * max_vel_x / 3)
                max_vel_theta = 2.5;
            else if ((std::abs(getPoseState().angular_velocity_) <= 2 &&
                      std::abs(getPoseState().angular_velocity_) > 1 * max_vel_x / 3) ||
                     (std::abs(getPoseState().velocity_) > 0.2 &&
                      std::abs(getPoseState().velocity_) <= 2 * max_vel_x / 3))
                max_vel_theta = 2;
            else
                max_vel_theta = 1.5;
        }
    }
}

void Robot_config::setRobotState(RobotState state) {
    currentState = state;

    switch (state) {
        case NORMAL_PLANNING:
            max_vel_x = 1.5;
            break;
        case LOW_SPEED_PLANNING:
            max_vel_x = 0.75;
            break;
        case NO_MAP_PLANNING:
            max_vel_x = 2.0;
            break;
        default:
            break;
    }
}

Robot_config::PoseState Robot_config::getPoseStateSafe() const {
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    return robot_state;
}

std::vector<double> Robot_config::getTimeIntervalSafe() const {
    std::lock_guard<std::mutex> lock(timeInterval_mutex_);
    return timeInterval;
}

std::vector<Eigen::Vector2f> Robot_config::getLaserDataSafe() const {
    std::lock_guard<std::mutex> lock(laser_data_mutex_);
    return laserData;
}

std::vector<double> Robot_config::getLaserDataDistanceSafe() const {
    std::lock_guard<std::mutex> lock(laser_data_mutex_);
    return laserDataDistance;
}

void Robot_config::getLocalGoalSafe(std::vector<double> &goal, std::vector<double> &goal_odom) const {
    std::lock_guard<std::mutex> lock(path_goal_mutex_);
    goal = local_goal;
    goal_odom = local_goal_odom;
}

void Robot_config::getObstacleDistanceSafe(double &front, double &latter) const {
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    front = front_obs;
    latter = latter_obs;
}

void Robot_config::publishRobotState() const {
    std_msgs::msg::String state_msg;
    switch (currentState) {
        case INITIALIZING: state_msg.data = "INITIALIZING"; break;
        case NORMAL_PLANNING: state_msg.data = "NORMAL_PLANNING"; break;
        case LOW_SPEED_PLANNING: state_msg.data = "LOW_SPEED_PLANNING"; break;
        case NO_MAP_PLANNING: state_msg.data = "NO_MAP_PLANNING"; break;
        case BRAKE_PLANNING: state_msg.data = "BRAKE_PLANNING"; break;
        case RECOVERY: state_msg.data = "RECOVERY"; break;
        case ROTATE_PLANNING: state_msg.data = "ROTATE_PLANNING"; break;
        case BACKWARD: state_msg.data = "BACKWARD"; break;
        case FORWARD: state_msg.data = "FORWARD"; break;
        case TEST: state_msg.data = "TEST"; break;
        case IDLE: state_msg.data = "IDLE"; break;
        default: state_msg.data = "UNKNOWN"; break;
    }
    robot_state_pub_->publish(state_msg);
}

void Robot_config::viewTrajectories(std::vector<PoseState> &trajectories, int nr_steps_, double theta_, std::vector<double> &t) const {
    if (!trajectory_pub_) return;

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "odom";

    int count = std::min(nr_steps_, static_cast<int>(trajectories.size()));
    for (int i = 0; i < count; ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = trajectories[i].x_;
        pose.pose.position.y = trajectories[i].y_;
        pose.pose.position.z = 0.0;

        double yaw = trajectories[i].theta_ + theta_;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(yaw / 2.0);
        pose.pose.orientation.w = std::cos(yaw / 2.0);

        path_msg.poses.push_back(pose);
    }

    trajectory_pub_->publish(path_msg);
    (void)t;  // Time interval not used in visualization
}

void Robot_config::viewTrajectories(std::vector<PoseState> &trajectories, int nr_steps_, std::vector<double> &t) const {
    viewTrajectories(trajectories, nr_steps_, 0.0, t);
}

void Robot_config::view_Goal(std::vector<double> &goal, std::vector<double> &goal1) const {
    if (!local_goal_pub_ || !global_goal_pub_) return;

    visualization_msgs::msg::Marker local_marker;
    local_marker.header.stamp = this->now();
    local_marker.header.frame_id = "odom";
    local_marker.ns = "local_goal";
    local_marker.id = 0;
    local_marker.type = visualization_msgs::msg::Marker::SPHERE;
    local_marker.action = visualization_msgs::msg::Marker::ADD;
    local_marker.scale.x = 0.2;
    local_marker.scale.y = 0.2;
    local_marker.scale.z = 0.2;
    local_marker.color.r = 0.0f;
    local_marker.color.g = 1.0f;
    local_marker.color.b = 0.0f;
    local_marker.color.a = 1.0f;

    if (goal.size() >= 2) {
        local_marker.pose.position.x = goal[0];
        local_marker.pose.position.y = goal[1];
        local_marker.pose.position.z = 0.1;
        local_goal_pub_->publish(local_marker);
    }

    visualization_msgs::msg::Marker global_marker = local_marker;
    global_marker.ns = "global_goal";
    global_marker.color.r = 1.0f;
    global_marker.color.g = 0.0f;

    if (goal1.size() >= 2) {
        global_marker.pose.position.x = goal1[0];
        global_marker.pose.position.y = goal1[1];
        global_marker.pose.position.z = 0.1;
        global_goal_pub_->publish(global_marker);
    }
}

void Robot_config::viewObstacles() const {
    // TODO: Implement obstacle visualization using PointCloud2
}
