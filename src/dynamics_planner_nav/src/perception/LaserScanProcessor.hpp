// LaserScanProcessor.hpp — 感知层：一帧激光的总处理入口(声明)
//
// perception 模块负责把原始传感器数据加工成上层决策能直接用的量。本文件把
// Go2_callbacks::laserScanCallback 里原本一大段「极坐标→odom 世界系点 + 8 方向边缘余量」
// 的处理整段收进来，回调只需调用 processLaserScan 一次，拿到结果再加锁写入即可。
//
// 内部复用 DirectionClearance 里的 selectEdgeFootprint / computeDirectionClearance，
// 各函数保持单一职责、可独立测试。本模块不依赖 Robot_config，只收普通参数，保持解耦。
//
// 实现见 LaserScanProcessor.cpp。

#ifndef DYNAMICS_PLANNER_NAV_PERCEPTION_LASER_SCAN_PROCESSOR_HPP
#define DYNAMICS_PLANNER_NAV_PERCEPTION_LASER_SCAN_PROCESSOR_HPP

#include <array>
#include <vector>

#include <sensor_msgs/msg/laser_scan.hpp>

#include "perception/DirectionClearance.hpp"   // kDirectionSectorCount + footprint/clearance 工具
#include "robot/Go2_footprint.hpp"

namespace perception {

// 一帧激光处理后的全部产物(供回调直接写入 Robot_config 对应字段)。
struct LaserScanResult {
    // 每个有效波束转到 odom 世界系后的障碍点 {x, y}(planner 通用格式)。
    std::vector<std::vector<double>> points_odom;
    // 与 points_odom 同序的【原始 range】(到雷达的标量距离，米)，非坐标。
    std::vector<double> ranges;
    // 机体系 8 方向(每 45°一扇区，0=前)到最近障碍的边缘余量(米)。含义见 DirectionClearance.hpp。
    std::array<double, kDirectionSectorCount> direction_clearance;
};

// 处理一帧激光：
//   1) 丢弃无效波束(超量程 / nan / inf)；
//   2) 把每个有效波束按机器人当前位姿(robot_x/y/theta)从极坐标转到 odom 世界系点；
//   3) 选一档代表机器人轮廓的 footprint，算出 8 方向边缘余量。
// dynamic_models / static_models 用于挑边缘 footprint(见 selectEdgeFootprint)。
LaserScanResult processLaserScan(
    const sensor_msgs::msg::LaserScan& scan,
    double robot_x, double robot_y, double robot_theta,
    const std::vector<go2::Footprint>& dynamic_models,
    const std::vector<go2::Footprint>& static_models);

}  // namespace perception

#endif  // DYNAMICS_PLANNER_NAV_PERCEPTION_LASER_SCAN_PROCESSOR_HPP

