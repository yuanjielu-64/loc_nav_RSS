// collision_publisher.cpp - ROS2 Humble version
// Publishes collision status from Gazebo

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <iostream>
#include <vector>
#include <string>

// Note: This is a simplified ROS2 version.
// For full Gazebo integration in ROS2, use ros_gz_bridge
// or gz_ros2_control packages.

class CollisionPublisher : public rclcpp::Node {
public:
    CollisionPublisher() : Node("collision_publisher"), is_colliding_(false), airborne_(false) {
        // Publisher for collision status
        pub_ = this->create_publisher<std_msgs::msg::Bool>("collision", 10);

        // Subscriber for odometry (to check if robot is airborne)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "ground_truth/state", 10,
            std::bind(&CollisionPublisher::position_callback, this, std::placeholders::_1));

        // Timer to publish at 50Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&CollisionPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Collision publisher started, publishing at 50Hz");

        // Note: For actual collision detection from Gazebo Fortress/Ignition,
        // you need to use gz-transport or ros_gz_bridge to subscribe to
        // /world/<world_name>/contact messages
    }

private:
    void position_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        airborne_ = (msg->pose.pose.position.z > 0.3);
    }

    void timer_callback() {
        auto collision_msg = std_msgs::msg::Bool();
        collision_msg.data = is_colliding_;
        pub_->publish(collision_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool is_colliding_;
    bool airborne_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CollisionPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
