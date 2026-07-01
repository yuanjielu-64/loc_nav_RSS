// LaserScanProcessor.cpp — 感知层：一帧激光的总处理入口(实现)
//
// 逐字搬自 Go2_callbacks::laserScanCallback 里原来的「极坐标→odom 点 + clearance」逻辑，
// 行为一致，只是从回调里抽出来独立成 perception 模块，让回调更短、更整洁。

#include "perception/LaserScanProcessor.hpp"

#include <cmath>

namespace perception {

LaserScanResult processLaserScan(
    const sensor_msgs::msg::LaserScan& scan,
    double robot_x, double robot_y, double robot_theta,
    const std::vector<go2::Footprint>& dynamic_models,
    const std::vector<go2::Footprint>& static_models) {

    LaserScanResult result;
    result.points_odom.reserve(scan.ranges.size());
    result.ranges.reserve(scan.ranges.size());

    // 后方自滤区(base_link 系，0=前)：狗屁股正后方一小块。狗端自滤偶尔漏掉自己的
    // 后腿/机身，这些点离机体很近且在正后方，落入此框就丢弃，避免把自身当障碍。
    //   px = 前向(负=身后)，py = 左向；只滤【近处】自身点，更远的真障碍仍保留。
    constexpr double kRearXNear = -0.30;   // 比此更靠后(px < 此值)才算"在身后"
    constexpr double kRearXFar  = -0.70;   // 只滤 [-0.70, -0.30] 这段近处，避免误删远障碍
    constexpr double kRearHalfWidth = 0.28;   // 横向半宽(m)
    auto isRearSelfPoint = [&](double angle, double range) {
        const double px = range * std::cos(angle);   // base_link 前向(0=前)
        const double py = range * std::sin(angle);   // base_link 左向
        return px < kRearXNear && px > kRearXFar && std::fabs(py) < kRearHalfWidth;
    };

    // /front/scan_filter 已由【狗端】完成自滤 + 下采样，这里只把有效波束从极坐标
    // 转到 odom 世界系。(range_min/max + nan/inf 检查只是丢弃无效波束，不属于"筛选"。)
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        const float range = scan.ranges[i];
        if (range < scan.range_min || range > scan.range_max ||
            std::isnan(range) || std::isinf(range)) {
            continue;
        }

        const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;

        // 后方自滤：丢弃落在机体正后方近处的自身点。
        if (isRearSelfPoint(angle, static_cast<double>(range))) continue;

        const double global_angle = angle + robot_theta;

        const double x = robot_x + range * std::cos(global_angle);
        const double y = robot_y + range * std::sin(global_angle);

        result.points_odom.push_back({x, y});
        result.ranges.push_back(static_cast<double>(range));
    }

    // 8 方向边缘余量：先选一档代表机器人轮廓的 footprint，再由它从这帧激光算出 clearance。
    const go2::Footprint edge_footprint = selectEdgeFootprint(dynamic_models, static_models);
    result.direction_clearance = computeDirectionClearance(scan, edge_footprint);

    return result;
}

}  // namespace perception

