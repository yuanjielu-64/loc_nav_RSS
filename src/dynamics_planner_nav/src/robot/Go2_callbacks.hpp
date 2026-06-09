// Go2_callbacks.hpp — ROS2 callback handlers
// Manages all ROS2 subscriber callbacks for Robot_config

#ifndef DYNAMICS_PLANNER_NAV_GO2_CALLBACKS_HPP
#define DYNAMICS_PLANNER_NAV_GO2_CALLBACKS_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

// Forward declaration
class Robot_config;

class Go2Callbacks {
public:
    explicit Go2Callbacks(Robot_config* robot);

    ~Go2Callbacks() = default;

    // ROS2 callback handlers (using SharedPtr messages)
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    void timeIntervalCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    void paramsCallback(const std_msgs::msg::String::SharedPtr msg);

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void velocityCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

private:
    Robot_config* robot_;  // Pointer to parent robot instance

    // Helper methods for globalPathCallback
    bool handleEmptyGlobalPath();
    void processValidGlobalPath(const nav_msgs::msg::Path::SharedPtr msg);
    double computeLookaheadThreshold() const;
};

#endif // DYNAMICS_PLANNER_NAV_GO2_CALLBACKS_HPP
