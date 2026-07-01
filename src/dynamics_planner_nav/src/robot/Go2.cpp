// Go2.cpp — Robot state manager and ROS2 interface
#include "Go2.hpp"
#include "Go2_callbacks.hpp"
#include <cmath>
#include <algorithm>   // std::transform(/force_state 大小写归一)
#include <limits>      // std::numeric_limits(8 方向 clearance 初值/比较)
#include <nav_msgs/srv/get_plan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "std_srvs/srv/empty.hpp"
#include "Utility.hpp"

// RobotState 枚举 -> 可读名字，供日志显示真实状态(而非裸数字 id)。
static const char* robotStateName(Robot_config::RobotState s) {
    switch (s) {
        case Robot_config::INIT:     return "INIT";
        case Robot_config::NORMAL:   return "NORMAL";
        case Robot_config::CAUTIOUS: return "CAUTIOUS";
        case Robot_config::BLIND:    return "BLIND";
        case Robot_config::BRAKE:    return "BRAKE";
        case Robot_config::RECOVER:  return "RECOVER";
        case Robot_config::ROTATE:   return "ROTATE";
        case Robot_config::BACK:     return "BACK";
        case Robot_config::FORWARD:  return "FORWARD";
        case Robot_config::TEST:     return "TEST";
        case Robot_config::IDLE:     return "IDLE";
        default:                     return "UNKNOWN";
    }
}

bool Robot_config::setup() {

    if (state_override_active_.load())
        setRobotState(state_override_value_);

    const PoseState pose = getPoseState();

    const bool gazebo_ok = !checkGazeboPaused();
    const bool state_ok  = getRobotState() != INIT;
    const bool pose_ok   = pose.valid_;
    const bool goal_ok = checkGoalReached(pose.x_, pose.y_);
    const bool map_ok = checkMapReady(goal_ok);
    const bool volume_ok = checkVolumeReady();

    const bool ready = gazebo_ok && state_ok && pose_ok && map_ok && goal_ok && volume_ok;

    // [setup] 就绪自检：每周期都打会刷屏、淹没有用日志 → 降为 DEBUG(默认隐藏)，
    // 需要排查就绪问题时把日志级别设为 DEBUG 即可调出。
    RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[setup] ready=%d | gazebo_ok=%d state_ok=%d(state=%s) pose_ok=%d "
        "map_ok=%d(map_src=%d laser=%zu costmap=%zu) goal_ok=%d volume_ok=%d",
        ready, gazebo_ok, state_ok, robotStateName(getRobotState()),
        pose_ok, map_ok, static_cast<int>(currentMap),
        laserData_odom.size(), costmapData.size(), goal_ok, volume_ok);

    return ready;
}


//==============================================================================
// Constructor: Initialize state and setup ROS2 communication
//==============================================================================
Robot_config::Robot_config(double bridge_max_velocity)
    : Node("dynamics_planner_nav"),
      algorithm(DWA),
      currentState(INIT),
      currentMap(ONLY_LASER_RECEIVED),
      local_goal_received(false),
      global_goal_received(false),
      param_received(false),
      canBeSolved(true),
      rotating_angle(0.0),
      dt(0.05),
      bridge_max_velocity_(bridge_max_velocity) {

    global_goal.reserve(3);
    global_goal_odom.reserve(3);
    local_goal.reserve(2);
    local_goal_odom.reserve(2);

    // 8 方向 clearance 初值设 INF：首帧激光到来前视为"四周无障碍"，避免被当成贴障。
    direction_clearance_.fill(std::numeric_limits<double>::infinity());

    // Initialize state
    local_goal = {0.0, 0.0};
    robot_state = PoseState(0.0, 0.0, 0.0, 0.0, 0.0, false);

    // Initialize time
    normal_to_low_time = this->now();
    low_to_normal_time = this->now();
    low_to_brake_time = this->now();
    recover_exit_time = this->now();

    // ---- Create Callback Handler ----
    callbacks_ = std::make_shared<Go2Callbacks>(this);

    // ---- Create Async Task Executor ----
    async_executor_ = std::make_shared<AsyncTaskExecutor>(num_threads);

    // ---- TF buffer/listener：按激光时间戳查 odom→base_link，修正运动时激光偏移 ----
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    robot_volume_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/robot_collision_models", 1,
        std::bind(&Go2Callbacks::robotVolumeCallback, callbacks_.get(), std::placeholders::_1));

    // Debug：/force_state 强制状态注入。值(不区分大小写)：
    //   NORMAL / CAUTIOUS(别名 LOW_SPEED/LOW) / RECOVER(别名 RECOVERY) -> 强制切到该状态并锁定；AUTO -> 恢复自动状态机。
    force_state_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/force_state", 1,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            std::string s = msg->data;
            std::transform(s.begin(), s.end(), s.begin(), ::toupper);
            if (s == "AUTO") {
                state_override_active_.store(false);
                RCLCPP_INFO(this->get_logger(), "[force_state] AUTO：恢复自动状态机");
            } else if (s == "NORMAL") {
                state_override_value_ = NORMAL;
                state_override_active_.store(true);
                RCLCPP_INFO(this->get_logger(), "[force_state] 锁定 NORMAL");
            } else if (s == "CAUTIOUS" || s == "LOW_SPEED" || s == "LOW") {
                state_override_value_ = CAUTIOUS;
                state_override_active_.store(true);
                RCLCPP_INFO(this->get_logger(), "[force_state] 锁定 CAUTIOUS");
            } else if (s == "RECOVER" || s == "RECOVERY") {
                state_override_value_ = RECOVER;
                state_override_active_.store(true);
                RCLCPP_INFO(this->get_logger(), "[force_state] 锁定 RECOVER");
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "[force_state] 未知值 '%s'(可用: NORMAL/CAUTIOUS/RECOVER/AUTO)", msg->data.c_str());
            }
        });

    robot_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/utlidar/robot_odom", rclcpp::SensorDataQoS(),
        std::bind(&Go2Callbacks::odometryCallback, callbacks_.get(), std::placeholders::_1));

    // 改订阅【狗端】已过滤的 /front/scan_filter(自滤+下采样已在狗上完成)，
    // PC 端 laserScanCallback 不再做任何筛选，直接转世界系坐标收入 laserData_odom。
    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/front/scan_filter", rclcpp::SensorDataQoS(),
        std::bind(&Go2Callbacks::laserScanCallback, callbacks_.get(), std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        std::bind(&Go2Callbacks::goalCallback, callbacks_.get(), std::placeholders::_1));

    costmap_update_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/local_costmap/costmap", 10,
        std::bind(&Go2Callbacks::costmapCallback, callbacks_.get(), std::placeholders::_1));

    velocity_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/utlidar/robot_odom", rclcpp::SensorDataQoS(),
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
    trajectory_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory", 10);
    global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);
    local_goal_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("local_goal", 1);
    global_goal_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("global_goal", 1);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    robot_state_pub_ = this->create_publisher<std_msgs::msg::String>("/robot_mode", 1);
    // RECOVER 脱困斥力可视化(箭头)：硬斥力(红) + 软斥力(橙)，发到 /recover_repulsion。
    repulsion_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/recover_repulsion", 1);
    // perception 激光点可视化：planner 实际使用的 laserData_odom(odom 系)，发到 /perception/laser_points。
    laser_points_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/perception/laser_points", 1);
    // 注意：/front/scan_filter 现由【狗端】发布(已过滤)。PC 端不再回发，避免同话题
    // 双发布者冲突；本节点改为订阅它(见 laser_scan_sub_)。

    // ---- ROS2 Service Clients ----
    global_path_clt_ = this->create_client<nav_msgs::srv::GetPlan>("/compute_path_to_pose");
    clear_costmaps_clt_ = this->create_client<std_srvs::srv::Empty>("/clear_costmaps");

    RCLCPP_INFO(this->get_logger(), "Robot_config initialized successfully");
    RCLCPP_INFO(this->get_logger(), "All planners will use %d parallel threads", num_threads);

    // Initialize tuning snapshot from current defaults
    tuning_params_ = getTuningParams();

    setStaticVolumes();
}


void Robot_config::setStaticVolumes() {
    auto built = go2::buildStaticVolumes(ROBOT_LENGTH, ROBOT_WIDTH, POINT_MASS_LENGTH);
    models_static_ = std::move(built);
}

void Robot_config::setRobotState(RobotState state) {
    currentState = state;
    setRobotVelocityLimits(state);
}

void Robot_config::setRobotVelocityLimits(RobotState state) {

    switch (state) {
        case NORMAL:
            max_vel_x = bridge_max_velocity_;          max_vel_y = 0.0;  max_vel_theta = 2.0;
            break;
        case CAUTIOUS:
            max_vel_x = bridge_max_velocity_ * 0.5;    max_vel_y = 0.25;  max_vel_theta = 2.5;
            break;
        case RECOVER:
            max_vel_x = bridge_max_velocity_ * 0.25;   max_vel_y = 0.5;  max_vel_theta = 3.0;
            break;
        case BLIND:
            max_vel_x = bridge_max_velocity_;          max_vel_y = 0.0;  max_vel_theta = 2.0;
            break;
        default:
            break;
    }
}

Robot_config::VelocityLimits Robot_config::getVelocityLimits() const {

    double back_ratio;
    switch (getRobotState()) {
        case RECOVER:  back_ratio = 0.0; break;   // 脱困 ddp 也禁止倒车：倒退仅用于潮汐推开，
                                                  // 那个用 publishCommand 直接下发，绕过本限制，不受影响。
        case CAUTIOUS: back_ratio = 0.0; break;
        default:       back_ratio = 0.0; break;   // NORMAL 及其它
    }
    return VelocityLimits{-back_ratio * bridge_max_velocity_, max_vel_x,
                          -max_vel_theta, max_vel_theta,
                          -max_vel_y, max_vel_y};
}

const std::vector<go2::Footprint>& Robot_config::getStaticVolumes() const {
    // models_static_ 构造后不可变(只在构造期 setStaticVolumes() 写一次)，返回 const 引用
    // 即可，省去每帧激光一次的整份 Footprint 向量拷贝，也无需加锁。
    return models_static_;
}

std::vector<std::vector<double>> Robot_config::getLaserData() const {
    std::lock_guard<std::mutex> lock(laser_data_mutex_);
    return laserData_odom;
}

Robot_config::PoseState Robot_config::getPoseStateAt(const rclcpp::Time &stamp) const {
    // 先拿最新位姿(含速度字段)作基底/回退。
    PoseState pose = getPoseState();

    if (!tf_buffer_) return pose;

    try {
        // 查激光采集时刻的 odom→base_link：用该时刻位姿变换激光，消除运动/转弯时的偏移。
        const auto tf = tf_buffer_->lookupTransform(
            "odom", "base_link", stamp, rclcpp::Duration::from_seconds(0.05));

        pose.x_ = tf.transform.translation.x;
        pose.y_ = tf.transform.translation.y;

        const auto &q = tf.transform.rotation;
        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        pose.theta_ = std::atan2(siny_cosp, cosy_cosp);
    } catch (const std::exception &) {
        // 查不到(超时/外推/TF 未就绪)：回退到最新位姿，不致命。
    }
    return pose;
}

std::vector<double> Robot_config::getTimeInterval() const {
    std::lock_guard<std::mutex> lock(timeInterval_mutex_);
    return timeInterval;
}

void Robot_config::getDirectionClearance(std::array<double, DIR_SECTOR_COUNT> &out) const {
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    out = direction_clearance_;
}

double Robot_config::mostOpenDirection(double &ux, double &uy) const {
    std::array<double, DIR_SECTOR_COUNT> clearance;
    getDirectionClearance(clearance);

    // 只在【已观测到】的方向里选最开阔的：余量为无穷大表示该扇区这一帧没有任何激光点，
    // 属于未知区域(例如真机前向激光看不到的身后)，朝那里挪等于盲走，必须排除。
    int best_direction = -1;
    double best_clearance = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < DIR_SECTOR_COUNT; ++i) {
        const double c = clearance[i];
        if (std::isinf(c)) continue;                 // 未观测方向：跳过
        if (c > best_clearance) { best_clearance = c; best_direction = i; }
    }

    // 所有方向都未观测(完全没有激光) → 没有可信的开阔方向，交给调用方原地转扫描。
    if (best_direction < 0) {
        ux = 0.0;
        uy = 0.0;
        return -1.0;
    }

    // 扇区 -> 机体系单位方向：方位角 = best_direction * 45°(CCW)。
    //   DIR_FRONT=0°(+x前) / DIR_LEFT=90°(+y左) / DIR_BACK=180°(-x后) / DIR_RIGHT=270°(-y右)。
    const double ang = static_cast<double>(best_direction) * (M_PI / 4.0);
    ux = std::cos(ang);
    uy = std::sin(ang);
    return best_clearance;   // 该方向余量(m)，供调用方判断是否四周都堵
}

double Robot_config::computeHardRepulsion(double hard_dist, double &fx, double &fy) const {
    std::array<double, DIR_SECTOR_COUNT> clearance;
    getDirectionClearance(clearance);

    fx = 0.0;
    fy = 0.0;
    for (int i = 0; i < DIR_SECTOR_COUNT; ++i) {
        const double c = clearance[i];
        // 比硬斥力边界远(含 +INF 无障碍)：不在硬区，不产生斥力。
        if (!(c < hard_dist)) continue;   // 用 !(c<hard) 同时挡掉 NaN

        // 越近(甚至 c<0 已侵入)，强度越大：weight = hard_dist − 余量 (>0)。
        const double weight = hard_dist - c;

        // 扇区 i → 机体系方位角(CCW，0=前)；斥力朝该方向的【反方向】= -(cos, sin)。
        const double ang = static_cast<double>(i) * (M_PI / 4.0);
        fx -= std::cos(ang) * weight;
        fy -= std::sin(ang) * weight;
    }
    return std::hypot(fx, fy);
}

double Robot_config::computeEscapeDirection(double goal_angle, double goal_gain,
                                            double &ux, double &uy) const {
    std::array<double, DIR_SECTOR_COUNT> clearance;
    getDirectionClearance(clearance);

    // 空旷度截顶(m)：远处/超大余量不让其无限主导，超过此值按此值计权。
    constexpr double kClearCap = 1.0;

    double ex = 0.0, ey = 0.0;
    for (int i = 0; i < DIR_SECTOR_COUNT; ++i) {
        const double c = clearance[i];
        if (std::isinf(c)) continue;                 // 未观测方向：朝那挪等于盲走，跳过(同 mostOpenDirection)
        // 空旷度权重：余量越大越想去；<0(已侵入)按 0，不往侵入方向走。
        const double w_open = std::min(std::max(c, 0.0), kClearCap);
        if (w_open <= 0.0) continue;
        // 扇区 i → 机体系方位角(CCW，0=前)。
        const double ang = static_cast<double>(i) * (M_PI / 4.0);
        // goal 门控引力：朝 goal 的方向额外加成(乘在空旷度上 → 朝障碍方向 w_open≈0，引力压不过避障)。
        const double w_goal = 1.0 + goal_gain * std::max(0.0, std::cos(ang - goal_angle));
        ex += std::cos(ang) * w_open * w_goal;
        ey += std::sin(ang) * w_open * w_goal;
    }

    const double mag = std::hypot(ex, ey);
    if (mag < 1e-6) { ux = 0.0; uy = 0.0; return 0.0; }   // 四周全未观测/无开阔 → 交调用方兜底
    ux = ex / mag;
    uy = ey / mag;
    return mag;
}





