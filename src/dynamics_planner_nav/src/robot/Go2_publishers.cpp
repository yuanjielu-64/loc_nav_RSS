// Go2_publishers.cpp — RViz 可视化实现
//
// go2::view  : 无状态纯构造函数(数据 -> ROS 消息)，便于复用与测试。
// Robot_config: viewTrajectories / view_Goal 在此作为薄包装(构造交给 go2::view，
//               发布用本类发布器)，让 Go2.cpp 更聚焦状态管理与 IO。
#include "Go2_publishers.hpp"
#include <algorithm>
#include <cmath>
namespace go2::view {
visualization_msgs::msg::MarkerArray makeTrajectory(
    const std::vector<Robot_config::PoseState> &traj,
    int steps, const rclcpp::Time &stamp) {
    visualization_msgs::msg::MarkerArray arr;
    const int n = std::min(steps, static_cast<int>(traj.size()));

    // 公共头部 + 把前 n 个轨迹点转成 geometry_msgs::Point(odom 系，抬高 z 便于观察)。
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = FRAME;
    std::vector<geometry_msgs::msg::Point> pts;
    pts.reserve(n);
    for (int i = 0; i < n; ++i) {
        geometry_msgs::msg::Point p;
        p.x = traj[i].x_;
        p.y = traj[i].y_;
        p.z = 0.1;
        pts.push_back(p);
    }

    // [0] 蓝色折线(LINE_STRIP)：把各点首尾相连成一条轨迹线。
    visualization_msgs::msg::Marker line;
    line.header = header;
    line.ns = "trajectory_line";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.03;                 // 线宽(m)
    line.color.r = 0.2f; line.color.g = 0.4f; line.color.b = 0.9f; line.color.a = 1.0f;  // 蓝
    line.pose.orientation.w = 1.0;
    line.points = pts;

    // [1] 轨迹点(SPHERE_LIST)：每个采样点画一个小球，亮青色以便从蓝线上区分。
    visualization_msgs::msg::Marker dots;
    dots.header = header;
    dots.ns = "trajectory_points";
    dots.id = 1;
    dots.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    dots.action = visualization_msgs::msg::Marker::ADD;
    dots.scale.x = dots.scale.y = dots.scale.z = 0.07;   // 小球直径(m)
    dots.color.r = 0.0f; dots.color.g = 0.9f; dots.color.b = 1.0f; dots.color.a = 1.0f;  // 青
    dots.pose.orientation.w = 1.0;
    dots.points = pts;

    arr.markers.push_back(std::move(line));
    arr.markers.push_back(std::move(dots));
    return arr;
}
visualization_msgs::msg::Marker makeSphere(const std::string &ns, double x, double y,
                                           float r, float g, float b,
                                           const rclcpp::Time &stamp) {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = FRAME;
    m.ns = ns;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = m.scale.y = m.scale.z = 0.2;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0f;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = 0.1;
    return m;
}

visualization_msgs::msg::Marker makeArrow(const std::string &ns, int id,
                                          double x, double y, double dx, double dy,
                                          float r, float g, float b,
                                          const rclcpp::Time &stamp) {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = FRAME;
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    // ARROW 用两点(起点→终点)表达方向：起点=机器人位置，终点=起点+世界系斥力向量。
    geometry_msgs::msg::Point p0, p1;
    p0.x = x;        p0.y = y;        p0.z = 0.15;
    p1.x = x + dx;   p1.y = y + dy;   p1.z = 0.15;
    m.points.push_back(p0);
    m.points.push_back(p1);
    m.scale.x = 0.04;   // 杆径(m)
    m.scale.y = 0.09;   // 箭头径(m)
    m.scale.z = 0.0;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0f;
    m.pose.orientation.w = 1.0;   // 用 points 表达几何，pose 设单位四元数
    return m;
}

visualization_msgs::msg::Marker makeLaserPoints(
    const std::vector<std::vector<double>> &pts, const rclcpp::Time &stamp) {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = FRAME;
    m.ns = "laser_points";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = m.scale.y = 0.05;   // 点的方块边长(m)
    m.color.r = 1.0f; m.color.g = 0.9f; m.color.b = 0.0f; m.color.a = 1.0f;   // 黄
    m.pose.orientation.w = 1.0;
    m.points.reserve(pts.size());
    for (const auto &p : pts) {
        if (p.size() < 2) continue;
        geometry_msgs::msg::Point pt;
        pt.x = p[0];
        pt.y = p[1];
        pt.z = 0.05;
        m.points.push_back(pt);
    }
    return m;
}
}  // namespace go2::view
// ---- Robot_config 可视化方法(薄包装) ----
void Robot_config::viewTrajectories(std::vector<PoseState> &trajectories, int nr_steps_) const {
    if (!trajectory_pub_) return;
    trajectory_pub_->publish(go2::view::makeTrajectory(trajectories, nr_steps_, this->now()));
}

void Robot_config::view_Goal(std::vector<double> &goal, std::vector<double> &goal1) const {
    if (!local_goal_pub_ || !global_goal_pub_) return;
    const rclcpp::Time stamp = this->now();
    if (goal.size() >= 2)   // local goal：绿
        local_goal_pub_->publish(go2::view::makeSphere("local_goal", goal[0], goal[1], 0, 1, 0, stamp));
    if (goal1.size() >= 2)  // global goal：红
        global_goal_pub_->publish(go2::view::makeSphere("global_goal", goal1[0], goal1[1], 1, 0, 0, stamp));
}

void Robot_config::viewRepulsion(double hard_fx, double hard_fy) const {
    if (!repulsion_pub_) return;

    const PoseState p = getPoseState();
    const rclcpp::Time stamp = this->now();
    const double c = std::cos(p.theta_), s = std::sin(p.theta_);

    // 机体系 (fx 前向, fy 左向) 旋到 odom 世界系。
    const double hdx = hard_fx * c - hard_fy * s;
    const double hdy = hard_fx * s + hard_fy * c;

    visualization_msgs::msg::MarkerArray arr;
    // 硬斥力 = 红箭头(id 0)。模长≈0 时箭头退化为点(不显眼)。
    arr.markers.push_back(
        go2::view::makeArrow("hard_repulsion", 0, p.x_, p.y_, hdx, hdy, 1.0f, 0.1f, 0.1f, stamp));
    repulsion_pub_->publish(arr);
}

void Robot_config::viewLaserPoints(const std::vector<std::vector<double>> &pts) const {
    if (!laser_points_pub_) return;
    laser_points_pub_->publish(go2::view::makeLaserPoints(pts, this->now()));
}

// 把当前状态机状态(currentState)发布到 /robot_mode(std_msgs/String)，供监控/可视化。
void Robot_config::publishRobotState() const {
    std_msgs::msg::String state_msg;
    switch (currentState.load()) {
        case INIT: state_msg.data = "INIT"; break;
        case NORMAL: state_msg.data = "NORMAL"; break;
        case CAUTIOUS: state_msg.data = "CAUTIOUS"; break;
        case BLIND: state_msg.data = "BLIND"; break;
        case BRAKE: state_msg.data = "BRAKE"; break;
        case RECOVER: state_msg.data = "RECOVER"; break;
        case ROTATE: state_msg.data = "ROTATE"; break;
        case BACK: state_msg.data = "BACK"; break;
        case FORWARD: state_msg.data = "FORWARD"; break;
        case TEST: state_msg.data = "TEST"; break;
        case IDLE: state_msg.data = "IDLE"; break;
        default: state_msg.data = "UNKNOWN"; break;
    }
    robot_state_pub_->publish(state_msg);
}

