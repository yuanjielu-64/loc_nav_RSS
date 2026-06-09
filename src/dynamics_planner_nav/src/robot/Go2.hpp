// Go2.hpp — Robot state/IO manager for Go2 navigation (ROS2 Humble)
// Manages ROS communication, state caching, and planner interface

#ifndef DYNAMICS_PLANNER_NAV_GO2_HPP
#define DYNAMICS_PLANNER_NAV_GO2_HPP

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/empty.hpp>

// Nav2 dependencies (replacing move_base)
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

// Other dependencies
#include <Eigen/Core>

// STL
#include <vector>
#include <memory>
#include <mutex>
#include <functional>

// Utils
#include "../utils/AsyncTaskExecutor.hpp"

// Forward declarations
class Go2Callbacks;

class Robot_config : public rclcpp::Node {
    // Friend classes for modular access
    friend class Go2Callbacks;

public:
    // Robot pose/velocity snapshot
    class PoseState {
    public:
        PoseState()
            : x_(0.0), y_(0.0), theta_(0.0), velocity_(0.0), angular_velocity_(0.0), valid_(false) {}

        PoseState(double x, double y, double theta, double v, double w, bool valid)
            : x_(x), y_(y), theta_(theta), velocity_(v), angular_velocity_(w), valid_(valid) {
        }

        std::vector<double> pose() const { return {x_, y_}; }

        double x_;
        double y_;
        double theta_;
        double velocity_;
        double angular_velocity_;
        bool valid_;
    };

    // Tuning parameters snapshot
    struct TuningParams {
        double max_vel_x;
        double max_vel_y;
        double max_vel_theta;
        int vx_sample;
        int vTheta_samples;
        double path_distance_bias;
        double goal_distance_bias;
        int nr_pairs_;
        int nr_steps_;
        double linear_stddev;
        double angular_stddev;
        double lambda;
        double local_goal_distance;
        double distance;
        double robot_radius_;
        double dt;
    };

    // Robot footprint (physical dimensions)
    struct Footprint {
        double length;
        double width;
    };

    // Velocity constraints for planning
    struct VelocityLimits {
        double min_linear;
        double max_linear;
        double min_angular;
        double max_angular;
    };

    // Obstacle placeholder
    class Obstacle {
    public:
        geometry_msgs::msg::Point center;
        double radius{};
    };

    // Supported local planners
    enum Algorithm {
        DWA,
        DWA_DDP,
        MPPI,
        MPPI_DDP,
        DDP,
        TEB,
        TEB_DDP
    };

    // High-level robot operating modes
    enum RobotState {
        INITIALIZING = 0,
        NORMAL_PLANNING = 1,
        LOW_SPEED_PLANNING = 2,
        NO_MAP_PLANNING = 3,
        BRAKE_PLANNING = 4,
        RECOVERY = 5,
        ROTATE_PLANNING = 6,
        BACKWARD = 7,
        FORWARD = 8,
        TEST = 9,
        IDLE = 10
    };

    // Active map source
    enum MapSource {
        ONLY_COSTMAP_RECEIVED = 0,
        ONLY_LASER_RECEIVED = 1,
        NO_ANY_RECEIVED = 2
    };

    Robot_config();
    ~Robot_config() = default;

    TuningParams getTuningParams() const;

    void setTuningParams(const TuningParams &tp);
    void setAlgorithm(Algorithm a) { algorithm = a; }
    void setDt(double t) { dt = t; }
    void setRobotState(RobotState state);
    void setLocalGoal(std::vector<double> &lg, double x, double y) {
        std::lock_guard<std::mutex> lock(path_goal_mutex_);
        local_goal = {lg[0], lg[1]};
        local_goal_odom = {x, y};
    }

    Algorithm getAlgorithm() const { return algorithm; }
    RobotState getRobotState() const { return currentState; }
    PoseState getPoseState() const { return robot_state; }
    double getDt() const { return dt; }
    double getVelocity() const { return robot_state.velocity_; }
    double getAngularVelocity() const { return robot_state.angular_velocity_; }
    bool getMapData();

    // Thread-safe getters
    PoseState getPoseStateSafe() const;
    std::vector<double> getTimeIntervalSafe() const;
    std::vector<Eigen::Vector2f> getLaserDataSafe() const;
    std::vector<double> getLaserDataDistanceSafe() const;
    void getLocalGoalSafe(std::vector<double> &goal, std::vector<double> &goal_odom) const;
    void getObstacleDistanceSafe(double &front, double &latter) const;

    std::vector<double> getLocalGoalCfg() { return local_goal; }
    std::vector<double> getGlobalGoalCfg() { return global_goal; }

    const std::vector<Eigen::Vector2f> &getLaserPoints() const { return laserData; }
    std::vector<std::vector<double>> getLaserData();
    std::vector<std::vector<double>> getCostmapDataOdom() const { return costmapDataOdom; }
    std::vector<std::vector<double>> getDataMap() { return map; }

    Footprint getFootprint() const;
    VelocityLimits getVelocityLimits() const;

    // In ROS2, costmap access pattern is different
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> getCostMapROS() { return costmap_ros_; }

    bool setup();
    bool checkGazeboPaused() const;

    void triggerRecovery();
    void resetStoppedStatus();
    void update_angular_velocity();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Control() { return cmd_vel_pub_; }
    void publishRobotState() const;
    void publishSmoothedPath(const std::vector<double> &xhat, const std::vector<double> &yhat) const;
    void publishTuningParams() const;

    void view_Goal(std::vector<double> &goal, std::vector<double> &goal1) const;
    void viewTrajectories(std::vector<PoseState> &trajectories, int nr_steps_, double theta_, std::vector<double> &t) const;
    void viewTrajectories(std::vector<PoseState> &trajectories, int nr_steps_, std::vector<double> &t) const;
    void viewObstacles() const;

    static double calculateTheta(const PoseState &state, const std::vector<double> &y);

    Algorithm algorithm;
    RobotState currentState;
    MapSource currentMap;

    bool local_goal_received{};
    bool global_goal_received{};
    bool param_received{};
    bool canBeSolved{};

    double rotating_angle;
    double dt{};
    double latter_obs{};
    double front_obs{};

    int recover_times = 0;
    int re = 1;
    int recover_to_low_count = 0;
    double dynamic_recovery_wait_time = 0.5;

    std::vector<std::vector<double>> global_paths;
    std::vector<std::vector<double>> global_paths_odom;
    std::vector<std::vector<double>> local_paths;
    std::vector<std::vector<double>> local_paths_odom;
    std::vector<std::vector<double>> local_goals_history;
    std::vector<std::vector<std::vector<double>>> global_paths_history;
    int history_index = 0;

    std::vector<std::vector<double>> actions;
    std::vector<double> local_goal_odom;
    std::vector<std::vector<double>> local_goal_point;
    std::vector<std::vector<double>> local_goal_point_odom;
    std::vector<double> global_goal_odom;

    std::vector<std::vector<geometry_msgs::msg::Point>> polygons;

    std::vector<std::vector<double>> costmapDataOdom;
    std::vector<std::vector<double>> costmapData;
    std::vector<double> laserDataDistance;

    std::vector<double> timeInterval = {
        0.0302, 0.0495, 0.0608, 0.0697, 0.0771, 0.0835, 0.0893, 0.0946, 0.0994, 0.1039,
        0.1082, 0.1122, 0.1160, 0.1196, 0.1231, 0.1264, 0.1296, 0.1327, 0.1357, 0.1386
    };

    // DWA parameters
    double max_vel_x = 1.5;
    double max_vel_y = 0.0;
    double max_vel_theta = 3.0;
    double vx_sample = 10;
    double vTheta_samples = 10;
    double path_distance_bias = 0.7;
    double goal_distance_bias = 0.7;

    // MPPI/DDP parameters
    double nr_pairs_ = 600;
    double nr_steps_ = 20;
    double linear_stddev = 0.1;
    double angular_stddev = 0.05;
    double lambda = 1.0;
    double local_goal_distance = 2.0;
    double distance = 0.3;
    double robot_radius_ = 0.01;

    int num_threads = 8;

    // ROS2 Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_global_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr local_goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr global_goal_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tuning_params_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;

    std::shared_ptr<Go2Callbacks> callbacks_;
    std::shared_ptr<AsyncTaskExecutor> async_executor_;

protected:
    // Thread safety
    mutable std::mutex robot_state_mutex_;
    mutable std::mutex timeInterval_mutex_;
    mutable std::mutex laser_data_mutex_;
    mutable std::mutex path_goal_mutex_;
    mutable std::mutex obstacle_mutex_;

    // ROS2 Subscriptions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_update_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr velocity_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr array_dt_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr params_sub_;

    // ROS2 Service Clients
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_costmaps_clt_;
    rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr global_path_clt_;

    // ROS2 Publishers (protected)
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_state_pub_;

    // Nav2 Costmap
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

    // Internal state
    TuningParams tuning_params_{};
    std::vector<double> global_goal;
    std::vector<double> local_goal;
    std::vector<double> costmapDataDistance;
    std::vector<double> costmapDataAngle;
    std::vector<Eigen::Vector2f> laserData;
    std::vector<std::vector<double>> map;

    PoseState robot_state;

    // State machine timing
    bool is_stopped = false;
    rclcpp::Time normal_to_low_time;
    bool normal_to_low_active = false;
    rclcpp::Time low_to_normal_time;
    bool low_to_normal_active = false;
    rclcpp::Time low_to_brake_time;
    bool low_to_brake_active = false;

    // Constants
    static constexpr double MIN_SPEED = 0.2;
    static constexpr double STOPPED_TIME_THRESHOLD = 1.0;
    static constexpr double ROBOT_LENGTH = 0.508;
    static constexpr double ROBOT_WIDTH = 0.430;
    static constexpr double POINT_MASS_LENGTH = 0.02;
    static constexpr double POINT_MASS_WIDTH = 0.02;
};

#endif // DYNAMICS_PLANNER_NAV_GO2_HPP
