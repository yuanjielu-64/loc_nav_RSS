// DirectionClearance.cpp — 感知层：从激光算【8 方向边缘余量】(实现)
//
// 逐字搬自 Go2_callbacks::laserScanCallback 里原来的 8 方向 clearance 逻辑，行为一致，
// 只是从回调里抽出来独立成 perception 模块。

#include "perception/DirectionClearance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace perception {

std::array<double, kDirectionSectorCount> computeDirectionClearance(
    const sensor_msgs::msg::LaserScan& scan,
    const go2::Footprint& edge_footprint) {

    std::array<double, kDirectionSectorCount> clearance;
    clearance.fill(std::numeric_limits<double>::infinity());

    // 拿不到机器人轮廓就无法把"中心距"换算成"边缘距"，全部保持未观测(+INF)。
    if (edge_footprint.empty())
        return clearance;

    const double sector_step = M_PI / 4.0;   // 每 45°一个扇区

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        const float range = scan.ranges[i];
        // 丢弃无效波束(超量程 / nan / inf)；这只是去无效值，不算"筛选"。
        if (range < scan.range_min || range > scan.range_max ||
            std::isnan(range) || std::isinf(range)) {
            continue;
        }

        // 方位角(相对机体前向，0=前)，归一化到 [-π, π]。
        const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
        const double normalized_angle = std::atan2(std::sin(angle), std::cos(angle));

        // 归入最近的 45°扇区(0..7)。
        const int sector =
            ((static_cast<int>(std::lround(normalized_angle / sector_step)) % kDirectionSectorCount)
             + kDirectionSectorCount) % kDirectionSectorCount;

        // 边缘距 = 中心距 − 该方位上机器人自身的边缘半径。
        const double edge_distance =
            static_cast<double>(range) - edge_footprint.edgeRadiusAtAngle(normalized_angle);

        if (edge_distance < clearance[sector])
            clearance[sector] = edge_distance;
    }

    return clearance;
}

go2::Footprint selectEdgeFootprint(
    const std::vector<go2::Footprint>& dynamic_models,
    const std::vector<go2::Footprint>& static_models) {

    // 优先用真实动态体积里 N=6 那档(索引 2)：比矩形更贴合狗的轮廓，边缘距更准。
    if (!dynamic_models.empty()) {
        const int idx = std::min<int>(2, static_cast<int>(dynamic_models.size()) - 1);
        return dynamic_models[idx];
    }

    // 回退静态矩形档(索引 2 = VOL_RECTANGLE，静态档里最精的一档)。
    constexpr size_t kStaticRectangleIndex = 2;
    if (static_models.size() > kStaticRectangleIndex &&
        !static_models[kStaticRectangleIndex].empty())
        return static_models[kStaticRectangleIndex];

    return go2::Footprint{};   // 都拿不到：空轮廓，clearance 将全为 +INF
}

}  // namespace perception
