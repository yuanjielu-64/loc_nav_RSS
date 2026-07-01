#include "Utility.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <rclcpp/clock.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
double l2_distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}
std::vector<double> savgolFilter(const std::vector<double> &data, int windowSize, int polyOrder) {
    if (windowSize % 2 == 0 || windowSize < 1)
        throw std::invalid_argument("Window size must be an odd positive integer.");
    if (polyOrder >= windowSize)
        throw std::invalid_argument("Polynomial order must be less than the window size.");
    int halfWindow = (windowSize - 1) / 2;
    Eigen::MatrixXd A(windowSize, polyOrder + 1);
    for (int i = 0; i < windowSize; ++i) {
        for (int j = 0; j <= polyOrder; ++j)
            A(i, j) = std::pow(i - halfWindow, j);
    }
    Eigen::VectorXd coeff = (A.transpose() * A).inverse() * A.transpose() * Eigen::VectorXd::Ones(windowSize);
    std::vector<double> filteredData(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        double smoothedValue = 0.0;
        for (int j = -halfWindow; j <= halfWindow; ++j) {
            int idx = std::clamp(static_cast<int>(i) + j, 0, static_cast<int>(data.size()) - 1);
            int coeffIndex = j + halfWindow;  // 应始终落在 [0, windowSize-1]
            if (coeffIndex < 0 || coeffIndex >= coeff.size())
                continue;  // 越界保护
            smoothedValue += coeff(coeffIndex) * data[idx];
        }
        filteredData[i] = smoothedValue;
    }
    return filteredData;
}
// ROS2 版：原 ROS1 的 geometry_msgs::PoseStamped / ros::Time::now() 已改为
// geometry_msgs::msg::PoseStamped / rclcpp::Clock。
geometry_msgs::msg::PoseStamped getPose(double x, double y, double theta) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, theta);
    pose.pose.orientation = tf2::toMsg(quaternion);
    return pose;
}
// world(odom) 点 (x,y) -> robot(base_link) 坐标，给定机器人位姿 (X,Y,PSI)。
// 刚体变换闭式解：
//   p_body = R(PSI)^T * (p_world - t),  其中 R(PSI)^T = [ c, s; -s, c]
// 比原先构造 3x3 矩阵 + inverse() 快一两个数量级；在长路径循环里收益明显。
std::vector<double> transform_lg(double x, double y, double X, double Y, double PSI) {
    const double c = std::cos(PSI);
    const double s = std::sin(PSI);
    const double dx = x - X;
    const double dy = y - Y;
    return { c * dx + s * dy,
            -s * dx + c * dy };
}
