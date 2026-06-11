// Go2_footprint.hpp — 机器人物理体积(几何)模型【声明】
//
// 把机器人的"物理体积"抽象成【体坐标系下若干形状的并集】。每个形状自带一个
// 有符号距离方程(SDF)：点在形状外为正、表面为 0、形状内为负。碰撞/距离代价
// 只需对所有形状取最小值即可。
//
// 默认就是一个简单的 Box（等价于原来的 Footprint{length,width}）。需要更精细
// 时再叠加 Circle / Capsule，或用 addCustom() 接入外部"预测体积"模型，也可以
// 用 FootprintReceiver 从 ROS topic 实时接收预测的形状。
//
// 实现见 Go2_footprint.cpp。坐标约定：体坐标系原点在 base_link，x 朝前、y 朝左。

#ifndef DYNAMICS_PLANNER_NAV_GO2_FOOTPRINT_HPP
#define DYNAMICS_PLANNER_NAV_GO2_FOOTPRINT_HPP

#include <algorithm>
#include <cmath>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace go2 {

struct Point2 {
    double x{0.0};
    double y{0.0};
};

// ---- 几何方程(SDF)：体坐标系下点 p 到形状表面的有符号距离(外正/内负) ----
double segmentDistance(const Point2& p, const Point2& a, const Point2& b);
double boxSDF(const Point2& p, const Point2& c, double hl, double hw,
              double cos_, double sin_);
double circleSDF(const Point2& p, const Point2& c, double r);
double capsuleSDF(const Point2& p, const Point2& a, const Point2& b, double r);

//==============================================================================
// Shape：统一的形状（值语义 + 类型标签）
//==============================================================================
struct Shape {
    enum Type { BOX, CIRCLE, CAPSULE, CUSTOM };

    Type type{BOX};
    Point2 center;            // BOX/CIRCLE 的中心，CAPSULE 的端点 a
    Point2 tip;               // CAPSULE 的端点 b
    double half_length{0.0};  // BOX 半长
    double half_width{0.0};   // BOX 半宽
    double radius{0.0};       // CIRCLE/CAPSULE 半径
    double cos_yaw{1.0};      // BOX 旋转
    double sin_yaw{0.0};

    // CUSTOM：由外部模型给出的任意 SDF。让"预测机器人体积"的模型把每周期
    // 算出的形状直接以函数形式接进来。
    std::function<double(const Point2&)> custom_sdf;
    double custom_extent{0.0};  // CUSTOM 的外接半径(给 inflation 参考)

    double signedDistance(const Point2& p) const;
    double maxExtent() const;   // 形状到体坐标原点的最大延伸(求外接圆用)
};

//==============================================================================
// Footprint：机器人完整物理体积 = 若干 Shape 的并集
//==============================================================================
class Footprint {
public:
    Footprint() = default;

    // ---- 链式添加形状 ----
    Footprint& addBox(double half_length, double half_width,
                      double cx = 0.0, double cy = 0.0, double yaw = 0.0);
    Footprint& addCircle(double radius, double cx = 0.0, double cy = 0.0);
    Footprint& addCapsule(const Point2& a, const Point2& b, double radius);
    // 接入"预测体积"模型：sdf(体坐标点)->有符号距离；extent 为外接半径。
    Footprint& addCustom(std::function<double(const Point2&)> sdf, double extent);

    void clear();
    bool empty() const;

    // 世界(odom)系障碍点到机器人体积的最小有符号距离。<0 表示已碰撞。
    double distanceToPoint(double rx, double ry, double rtheta,
                           double world_x, double world_y) const;
    // 是否碰撞（有符号距离 <= margin）
    bool isColliding(double rx, double ry, double rtheta,
                     double world_x, double world_y, double margin = 0.0) const;
    // 外接圆半径（给 costmap inflation_radius 参考）
    double circumscribedRadius() const;

    // ---- 预设模型 ----
    static Footprint simpleBox(double length, double width);      // 默认：单个矩形
    static Footprint pointMass(double size = 0.02);               // 点质量(倒车/无图)
    static Footprint compound(double body_len, double body_wid,   // 机身 box + 四角圆
                              double leg_r, double leg_dx, double leg_dy);
    static Footprint capsuleBody(double half_len, double radius); // 沿 x 的胶囊机身

    // ---- 从 ROS Float64MultiArray 解析(见 .cpp 的编码说明) ----
    static Footprint fromFloatArray(const std_msgs::msg::Float64MultiArray& msg);

private:
    std::vector<Shape> shapes_;
};

//==============================================================================
// FootprintReceiver：订阅一个 ROS topic，实时接收"预测的机器人体积"。
// 现在没有发布者也能正常存在；未来你的预测模型往该 topic 发 Float64MultiArray
// 即可。线程安全：回调写、getLatest() 读，用互斥量保护。
//==============================================================================
class FootprintReceiver {
public:
    // node：宿主节点；topic：预测体积话题(默认 /predicted_footprint)。
    FootprintReceiver(rclcpp::Node* node,
                      const std::string& topic = "/predicted_footprint");

    bool hasReceived() const;     // 是否收到过至少一帧预测
    Footprint getLatest() const;  // 取最新预测的 footprint(线程安全拷贝)

private:
    void callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
    mutable std::mutex mutex_;
    Footprint latest_;
    bool received_{false};
};

}  // namespace go2

#endif  // DYNAMICS_PLANNER_NAV_GO2_FOOTPRINT_HPP

