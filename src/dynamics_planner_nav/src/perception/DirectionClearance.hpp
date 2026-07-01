// DirectionClearance.hpp — 感知层：从激光算【8 方向边缘余量】(声明)
//
// perception 模块负责把原始传感器数据(这里是 LaserScan)处理成上层决策能直接用的
// 几何感知量。本文件提供「机体系 8 个方向(每 45°一扇区)到最近障碍的边缘余量」计算，
// 原先内联在 Go2_callbacks::laserScanCallback 里，抽出来让回调只管收激光点、更整洁。
//
// 实现见 DirectionClearance.cpp。

#ifndef DYNAMICS_PLANNER_NAV_PERCEPTION_DIRECTION_CLEARANCE_HPP
#define DYNAMICS_PLANNER_NAV_PERCEPTION_DIRECTION_CLEARANCE_HPP

#include <array>
#include <vector>

#include <sensor_msgs/msg/laser_scan.hpp>

#include "robot/Go2_footprint.hpp"

namespace perception {

// 机体系方向扇区数：8 个(每 45°一个，0=正前，CCW 递增)。与 Robot_config::DirSector 对齐。
constexpr int kDirectionSectorCount = 8;

// 从一帧激光算【8 方向边缘余量】(米)：
//   每束有效激光按方位角(机体系，0=前)归入最近的 45°扇区，取该扇区上
//   (中心距 range − 该方位机器人边缘半径) 的最小值。
//   值 >0=还有余量，<0=已侵入机身，+INF=该扇区这一帧没观测到障碍。
//   edge_footprint 为空(拿不到机器人轮廓)时无法换算边缘半径，返回全 +INF。
std::array<double, kDirectionSectorCount> computeDirectionClearance(
    const sensor_msgs::msg::LaserScan& scan,
    const go2::Footprint& edge_footprint);

// 挑一档代表机器人边缘轮廓的 footprint(用于把激光"中心距"换算成"边缘距")：
//   优先真实动态体积里 N=6 那档(索引 2，最贴合狗轮廓)，拿不到则回退静态矩形档
//   (索引 2 = VolumeModel::VOL_RECTANGLE)。两者都拿不到则返回空 Footprint。
go2::Footprint selectEdgeFootprint(
    const std::vector<go2::Footprint>& dynamic_models,
    const std::vector<go2::Footprint>& static_models);

}  // namespace perception

#endif  // DYNAMICS_PLANNER_NAV_PERCEPTION_DIRECTION_CLEARANCE_HPP
