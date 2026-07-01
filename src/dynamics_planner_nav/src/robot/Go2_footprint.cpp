// Go2_footprint.cpp — 机器人物理体积(几何)模型【实现】
//
// 几何方程(SDF)、Shape/Footprint 的方法都实现在这里。声明见 Go2_footprint.hpp。
// 体积档的存储现在直接放在 Robot_config 上(models_static_ / models_dynamic_)，
// ROS 订阅解析则在 Go2_callbacks::robotVolumeCallback 里就地完成。

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

double polygonSDF(const Point2& p, const std::vector<Point2>& verts) {
    const size_t n = verts.size();
    if (n == 0) return std::numeric_limits<double>::max();
    if (n == 1) return std::hypot(p.x - verts[0].x, p.y - verts[0].y);
    if (n == 2) return segmentDistance(p, verts[0], verts[1]);

    // 到最近边的距离 + 凸多边形内外判定(叉积同号，与绕向无关)。
    double dmin = std::numeric_limits<double>::max();
    bool allPos = true, allNeg = true;
    for (size_t i = 0; i < n; ++i) {
        const Point2& a = verts[i];
        const Point2& b = verts[(i + 1) % n];
        dmin = std::min(dmin, segmentDistance(p, a, b));
        const double cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        if (cross < 0.0) allPos = false;
        if (cross > 0.0) allNeg = false;
    }
    const bool inside = allPos || allNeg;
    return inside ? -dmin : dmin;
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
        case POLYGON: return polygonSDF(p, verts);
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
        case POLYGON: {
            double r = 0.0;
            for (const auto& v : verts) r = std::max(r, std::hypot(v.x, v.y));
            return r;
        }
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

Footprint& Footprint::addPolygon(const std::vector<Point2>& verts) {
    Shape s;
    s.type = Shape::POLYGON;
    s.verts = verts;
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

double Footprint::distanceToSegment(double rxA, double ryA, double rthetaA,
                                    double rxB, double ryB, double rthetaB,
                                    double world_x, double world_y,
                                    double d_max, double a_max) const {
    const double dx = rxB - rxA;
    const double dy = ryB - ryA;
    const double trans = std::hypot(dx, dy);

    // 转角差归一化到[-pi,pi]，走最短弧插值(避免 359°->1° 当成转一大圈)。
    double dth = rthetaB - rthetaA;
    while (dth >  M_PI) dth -= 2.0 * M_PI;
    while (dth < -M_PI) dth += 2.0 * M_PI;

    // 自适应子步：平移和转角各自所需细分取较大者；近场才会被调用，K 通常 1~3。
    const int k_trans = (d_max > 0.0) ? static_cast<int>(std::ceil(trans / d_max)) : 1;
    const int k_rot   = (a_max > 0.0) ? static_cast<int>(std::ceil(std::fabs(dth) / a_max)) : 1;
    const int K = std::max({1, k_trans, k_rot});

    double best = std::numeric_limits<double>::max();
    for (int k = 0; k <= K; ++k) {
        const double s  = static_cast<double>(k) / K;
        const double rx = rxA + s * dx;
        const double ry = ryA + s * dy;
        const double rt = rthetaA + s * dth;
        best = std::min(best, distanceToPoint(rx, ry, rt, world_x, world_y));
    }
    return best;
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

double Footprint::edgeRadiusAtAngle(double angle) const {
    if (shapes_.empty()) return 0.0;

    const double dx = std::cos(angle);
    const double dy = std::sin(angle);
    // 体坐标系下点 (t*dir) 到机器人体积的有符号距离：内部<0, 外部>0。
    auto sdAt = [&](double t) {
        const Point2 p{t * dx, t * dy};
        double d = std::numeric_limits<double>::max();
        for (const auto& s : shapes_) d = std::min(d, s.signedDistance(p));
        return d;
    };

    // 原点必须在体积内部(star-shaped 前提)，否则该方向无良定义边缘半径。
    if (sdAt(0.0) > 0.0) return 0.0;

    double hi = circumscribedRadius() + 1e-3;  // 上界：外接圆外一点，必在体外
    if (sdAt(hi) < 0.0) return hi;             // 兜底(理论不会发生)

    // 二分找零点(最外层边界)：lo 始终在内部, hi 始终在外部。
    double lo = 0.0;
    for (int i = 0; i < 40; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (sdAt(mid) < 0.0) lo = mid; else hi = mid;
    }
    return 0.5 * (lo + hi);
}

std::vector<double> Footprint::radialProfile(int n) const {
    std::vector<double> prof;
    if (n <= 0) return prof;
    prof.reserve(n);
    for (int i = 0; i < n; ++i) {
        prof.push_back(edgeRadiusAtAngle(2.0 * M_PI * i / n));
    }
    return prof;
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

std::vector<Footprint> Footprint::parseFootprints(const std_msgs::msg::Float64MultiArray& msg) {
    // 编码: data[0]=档数; 每档 [kind,count,payload]:
    //   kind=1 圆     -> payload = cx,cy,r              (count 恒为 1)
    //   kind=0 多边形 -> payload = count 个 (x,y)        (base_link 系，已含 margin)
    std::vector<Footprint> out;
    const auto& d = msg.data;
    if (d.empty()) return out;

    const int num_models = static_cast<int>(d[0]);
    size_t i = 1;
    for (int m = 0; m < num_models && i + 1 < d.size(); ++m) {
        const int kind = static_cast<int>(d[i]);
        const int count = static_cast<int>(d[i + 1]);
        i += 2;
        Footprint f;
        if (kind == 1) {
            // 圆: cx, cy, r
            if (i + 3 > d.size()) break;
            f.addCircle(d[i + 2], d[i + 0], d[i + 1]);
            i += 3;
        } else {
            // 多边形: count 个 (x,y)
            if (i + static_cast<size_t>(2 * count) > d.size()) break;
            std::vector<Point2> verts;
            verts.reserve(count);
            for (int k = 0; k < count; ++k) {
                verts.push_back(Point2{d[i + 2 * k], d[i + 2 * k + 1]});
            }
            f.addPolygon(verts);
            i += static_cast<size_t>(2 * count);
        }
        out.push_back(std::move(f));
    }
    return out;
}


//==============================================================================
// buildStaticVolumes：按机身尺寸构建静态体积档位(几何实现集中在此，便于扩展)。
// 返回顺序固定，索引与 Robot_config::VolumeModel 对齐：[0]质点 [1]外接圆 [2]矩形。
//==============================================================================
std::vector<Footprint> buildStaticVolumes(double length, double width, double point_mass) {
    const double hl = length / 2.0;
    const double hw = width / 2.0;

    std::vector<Footprint> models(3);

    // [0] 质点
    models[0] = Footprint::pointMass(point_mass);
    // [1] 粗糙：单个外接圆(最保守)，半径 = 机身半对角线
    models[1].addCircle(std::hypot(hl, hw));
    // [2] 正常：矩形
    models[2] = Footprint::simpleBox(length, width);

    return models;
}

}  // namespace go2



