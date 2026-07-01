// Go2_publishers.hpp — RViz 可视化模块
//
// 把"只为 RViz 服务"的绘制从 Go2.cpp 抽出来集中放这里。go2::view 下是一组无状态的
// 纯构造函数(数据 + 时间戳 -> ROS 消息)，真正的 publish 仍由 Robot_config 的发布器完成
// (薄包装见 Go2_publishers.cpp 里的 Robot_config::viewTrajectories / view_Goal)。
// 以后新增可视化(8 方向 clearance、footprint 轮廓等)在 go2::view 里加 make* 即可。

#ifndef DYNAMICS_PLANNER_NAV_GO2_PUBLISHERS_HPP
#define DYNAMICS_PLANNER_NAV_GO2_PUBLISHERS_HPP

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "Go2.hpp"   // Robot_config::PoseState

namespace go2::view {

inline constexpr char FRAME[] = "odom";   // 可视化统一发布在 odom 帧

// 轨迹(odom 系) -> MarkerArray：取前 steps 个点，生成两个 Marker——
//   [0] LINE_STRIP：把各点连成一条蓝线；
//   [1] SPHERE_LIST：在每个轨迹点画一个小球，便于看清离散采样点。
// 只画位置(不画朝向)，故无需 yaw。
visualization_msgs::msg::MarkerArray makeTrajectory(
    const std::vector<Robot_config::PoseState> &traj,
    int steps, const rclcpp::Time &stamp);

// 目标点 -> 球形 Marker：ns 区分 local/global，颜色用 (r,g,b)。
visualization_msgs::msg::Marker makeSphere(const std::string &ns, double x, double y,
                                           float r, float g, float b,
                                           const rclcpp::Time &stamp);

// 箭头 Marker：从 (x,y)(odom 起点) 指向 (x+dx, y+dy)(odom 世界系增量向量)，用 (r,g,b) 上色。
//   用 points[0]=起点、points[1]=终点 表示方向；scale.x=杆径, scale.y=箭头径。
//   斥力可视化用：调用方先把机体系斥力旋到世界系得 (dx,dy) 再传入。
visualization_msgs::msg::Marker makeArrow(const std::string &ns, int id,
                                          double x, double y, double dx, double dy,
                                          float r, float g, float b,
                                          const rclcpp::Time &stamp);

// perception 处理后的激光点(odom 系，planner 实际使用的 laserData_odom) -> POINTS Marker。
//   每个点画一个小方块，便于在 RViz 里核对感知到的障碍点是否正确。
visualization_msgs::msg::Marker makeLaserPoints(
    const std::vector<std::vector<double>> &pts, const rclcpp::Time &stamp);

}  // namespace go2::view

#endif  // DYNAMICS_PLANNER_NAV_GO2_PUBLISHERS_HPP

