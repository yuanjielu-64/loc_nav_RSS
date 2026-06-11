// Go2_footprint.cpp — 机器人物理体积(几何)模型【实现】
//
// 几何方程(SDF)、Shape/Footprint 的方法、以及从 ROS topic 接收"预测体积"的
// FootprintReceiver 都实现在这里。声明见 Go2_footprint.hpp。

#include "Go2_footprint.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace go2 {

//==============================================================================
// 几何方程(SDF)
//==============================================================================

double segmentDistance(const Point2& p, const Point2& a, const Point2& b) {
    const double abx = b.x - a.x;
    const double aby = b.y - a.y;
    const double ab2 = abx * abx + aby * aby;
    double t = 0.0;
    if (ab2 > 1e-12) {
        t = std::clamp(((p.x - a.x) * abx + (p.y - a.y) * aby) / ab2, 0.0, 1.0);
    }
    return std::hypot(p.x - (a.x + t * abx), p.y - (a.y + t * aby));
}

double boxSDF(const Point2& p, const Point2& c, double hl, double hw,
              double cos_, double sin_) {
    const double rx = p.x - c.x;
    const double ry = p.y - c.y;
    const double lx = cos_ * rx + sin_ * ry;   // 旋到盒子局部系
    const double ly = -sin_ * rx + cos_ * ry;
    const double dx = std::fabs(lx) - hl;
    const double dy = std::fabs(ly) - hw;
    const double outside = std::hypot(std::max(dx, 0.0), std::max(dy, 0.0));
    const double inside = std::min(std::max(dx, dy), 0.0);
    return outside + inside;
}

double circleSDF(const Point2& p, const Point2& c, double r) {
    return std::hypot(p.x - c.x, p.y - c.y) - r;
}

double capsuleSDF(const Point2& p, const Point2& a, const Point2& b, double r) {
    return segmentDistance(p, a, b) - r;
}

//==============================================================================
// Shape
//==============================================================================

double Shape::signedDistance(const Point2& p) const {
    switch (type) {
        case BOX:     return boxSDF(p, center, half_length, half_width, cos_yaw, sin_yaw);
        case CIRCLE:  return circleSDF(p, center, radius);
        case CAPSULE: return capsuleSDF(p, center, tip, radius);
        case CUSTOM:  return custom_sdf ? custom_sdf(p)
                                        : std::numeric_limits<double>::max();
    }
    return std::numeric_limits<double>::max();
}

double Shape::maxExtent() const {
    switch (type) {
        case BOX:     return std::hypot(center.x, center.y) + std::hypot(half_length, half_width);
        case CIRCLE:  return std::hypot(center.x, center.y) + radius;
        case CAPSULE: return std::max(std::hypot(center.x, center.y),
                                      std::hypot(tip.x, tip.y)) + radius;
        case CUSTOM:  return custom_extent;
    }
    return 0.0;
}

//==============================================================================
// Footprint — 添加形状
//==============================================================================

Footprint& Footprint::addBox(double half_length, double half_width,
                             double cx, double cy, double yaw) {
    Shape s;
    s.type = Shape::BOX;
    s.center = {cx, cy};
    s.half_length = half_length;
    s.half_width = half_width;
    s.cos_yaw = std::cos(yaw);
    s.sin_yaw = std::sin(yaw);
    shapes_.push_back(std::move(s));
    return *this;
}

Footprint& Footprint::addCircle(double radius, double cx, double cy) {
    Shape s;
    s.type = Shape::CIRCLE;
    s.center = {cx, cy};
    s.radius = radius;
    shapes_.push_back(std::move(s));
    return *this;
}

Footprint& Footprint::addCapsule(const Point2& a, const Point2& b, double radius) {
    Shape s;
    s.type = Shape::CAPSULE;
    s.center = a;
    s.tip = b;
    s.radius = radius;
    shapes_.push_back(std::move(s));
    return *this;
}

Footprint& Footprint::addCustom(std::function<double(const Point2&)> sdf, double extent) {
    Shape s;
    s.type = Shape::CUSTOM;
    s.custom_sdf = std::move(sdf);
    s.custom_extent = extent;
    shapes_.push_back(std::move(s));
    return *this;
}

void Footprint::clear() { shapes_.clear(); }

bool Footprint::empty() const { return shapes_.empty(); }

//==============================================================================
// Footprint — 查询
//==============================================================================

double Footprint::distanceToPoint(double rx, double ry, double rtheta,
                                  double world_x, double world_y) const {
    // 世界系 -> 体坐标系：平移到机器人，再绕 -theta 旋转
    const double relx = world_x - rx;
    const double rely = world_y - ry;
    const double c = std::cos(rtheta);
    const double s = std::sin(rtheta);
    const Point2 local{c * relx + s * rely, -s * relx + c * rely};

    double d = std::numeric_limits<double>::max();
    for (const auto& shape : shapes_) {
        d = std::min(d, shape.signedDistance(local));
    }
    return d;
}

bool Footprint::isColliding(double rx, double ry, double rtheta,
                            double world_x, double world_y, double margin) const {
    return distanceToPoint(rx, ry, rtheta, world_x, world_y) <= margin;
}

double Footprint::circumscribedRadius() const {
    double r = 0.0;
    for (const auto& shape : shapes_) r = std::max(r, shape.maxExtent());
    return r;
}

//==============================================================================
// Footprint — 预设模型
//==============================================================================

Footprint Footprint::simpleBox(double length, double width) {
    Footprint f;
    f.addBox(length * 0.5, width * 0.5);
    return f;
}

Footprint Footprint::pointMass(double size) {
    Footprint f;
    f.addBox(size * 0.5, size * 0.5);
    return f;
}

Footprint Footprint::compound(double body_len, double body_wid,
                              double leg_r, double leg_dx, double leg_dy) {
    Footprint f;
    f.addBox(body_len * 0.5, body_wid * 0.5);
    f.addCircle(leg_r,  leg_dx,  leg_dy);
    f.addCircle(leg_r,  leg_dx, -leg_dy);
    f.addCircle(leg_r, -leg_dx,  leg_dy);
    f.addCircle(leg_r, -leg_dx, -leg_dy);
    return f;
}

Footprint Footprint::capsuleBody(double half_len, double radius) {
    Footprint f;
    f.addCapsule(Point2{-half_len, 0.0}, Point2{half_len, 0.0}, radius);
    return f;
}

//==============================================================================
// Footprint — 从 ROS Float64MultiArray 解析
//
// 编码：扁平数组，每 7 个 double 描述一个形状 [type, p0, p1, p2, p3, p4, p5]
//   type 0 = BOX:     p0=half_length p1=half_width p2=cx p3=cy p4=yaw
//   type 1 = CIRCLE:  p0=radius p1=cx p2=cy
//   type 2 = CAPSULE: p0=ax p1=ay p2=bx p3=by p4=radius
// 多个形状就把多段 7-元组接在一起。长度不是 7 的倍数时，忽略末尾不完整的一段。
//==============================================================================

Footprint Footprint::fromFloatArray(const std_msgs::msg::Float64MultiArray& msg) {
    constexpr size_t kRecord = 7;
    Footprint f;
    const auto& d = msg.data;
    for (size_t i = 0; i + kRecord <= d.size(); i += kRecord) {
        const int type = static_cast<int>(d[i]);
        switch (type) {
            case Shape::BOX:
                f.addBox(d[i + 1], d[i + 2], d[i + 3], d[i + 4], d[i + 5]);
                break;
            case Shape::CIRCLE:
                f.addCircle(d[i + 1], d[i + 2], d[i + 3]);
                break;
            case Shape::CAPSULE:
                f.addCapsule(Point2{d[i + 1], d[i + 2]}, Point2{d[i + 3], d[i + 4]}, d[i + 5]);
                break;
            default:
                break;  // 未知类型(含 CUSTOM，无法用纯数据表达)直接跳过
        }
    }
    return f;
}

//==============================================================================
// FootprintReceiver — ROS 订阅器
//==============================================================================

FootprintReceiver::FootprintReceiver(rclcpp::Node* node, const std::string& topic) {
    sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        topic, 1,
        std::bind(&FootprintReceiver::callback, this, std::placeholders::_1));
}

void FootprintReceiver::callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    Footprint parsed = Footprint::fromFloatArray(*msg);
    if (parsed.empty()) return;  // 空/无效预测：保留上一帧，不覆盖
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = std::move(parsed);
    received_ = true;
}

bool FootprintReceiver::hasReceived() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return received_;
}

Footprint FootprintReceiver::getLatest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
}

}  // namespace go2



