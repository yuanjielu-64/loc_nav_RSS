// Go2_footprint.hpp — 机器人物理体积(几何)模型【声明】
//
// 把机器人的"物理体积"抽象成【体坐标系下若干形状的并集】。每个形状自带一个
// 有符号距离方程(SDF)：点在形状外为正、表面为 0、形状内为负。碰撞/距离代价
// 只需对所有形状取最小值即可。
//
// 默认就是一个简单的 Box（等价于原来的 Footprint{length,width}）。需要更精细
// 时再叠加 Circle / Capsule，或用 addCustom() 接入外部"预测体积"模型。运行时的
// 体积接收(/predicted_footprint、/robot_collision_models)直接存在 Robot_config 的
// models_dynamic_ 上，ROS 订阅回调统一放在 Go2_callbacks。
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
// 凸多边形有符号距离(外正/内负)。verts 为顶点(任意绕向)；内外判定用叉积同号(凸前提)。
double polygonSDF(const Point2& p, const std::vector<Point2>& verts);

//==============================================================================
// Shape：统一的形状（值语义 + 类型标签）
//==============================================================================
struct Shape {
    enum Type { BOX, CIRCLE, CAPSULE, CUSTOM, POLYGON };

    Type type{BOX};
    Point2 center;            // BOX/CIRCLE 的中心，CAPSULE 的端点 a
    Point2 tip;               // CAPSULE 的端点 b
    double half_length{0.0};  // BOX 半长
    double half_width{0.0};   // BOX 半宽
    double radius{0.0};       // CIRCLE/CAPSULE 半径
    double cos_yaw{1.0};      // BOX 旋转
    double sin_yaw{0.0};
    std::vector<Point2> verts;  // POLYGON 顶点(体坐标系，来自 /robot_collision_models)

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
    // 凸多边形(来自 /robot_collision_models 的某一档，base_link 系顶点)。
    Footprint& addPolygon(const std::vector<Point2>& verts);
    // 接入"预测体积"模型：sdf(体坐标点)->有符号距离；extent 为外接半径。
    Footprint& addCustom(std::function<double(const Point2&)> sdf, double extent);

    void clear();
    bool empty() const;

    // 世界(odom)系障碍点到机器人体积的最小有符号距离。<0 表示已碰撞。
    double distanceToPoint(double rx, double ry, double rtheta,
                           double world_x, double world_y) const;
    // 整段运动(poseA->poseB)对世界系障碍点的最小有符号距离：在两姿态之间按
    // 平移 d_max / 转角 a_max 自适应插入子步，逐个 distanceToPoint 取最小，从而
    // 连续覆盖【整条线段】(而不是只检查端点)，同时把成本控制在少数子步上。
    //   子步数 K = max(1, ceil(平移/d_max), ceil(|Δθ|/a_max))，姿态线性插值、
    //   角度走最短弧。仅在近场(粗筛未通过)调用，远场无需进来。
    double distanceToSegment(double rxA, double ryA, double rthetaA,
                             double rxB, double ryB, double rthetaB,
                             double world_x, double world_y,
                             double d_max = 0.1, double a_max = 0.15) const;
    // 是否碰撞（有符号距离 <= margin）
    bool isColliding(double rx, double ry, double rtheta,
                     double world_x, double world_y, double margin = 0.0) const;
    // 外接圆半径（给 costmap inflation_radius 参考）
    double circumscribedRadius() const;

    // ---- 极坐标轮廓：以机器人为中心，x 轴朝前(angle=0)，CCW 为正 ----
    // edgeRadiusAtAngle: 沿 angle 方向从中心到机器人边缘的距离(机器人在该方向的"半径")。
    //   用途：障碍物间距 ≈ 障碍物到中心距离 − edgeRadiusAtAngle(障碍物方位角)。
    double edgeRadiusAtAngle(double angle) const;
    // radialProfile: 均匀采样 n 个角度(覆盖[0,2π))，返回整条边缘半径剖面(一次算、反复用)。
    std::vector<double> radialProfile(int n) const;

    // ---- 预设模型 ----
    static Footprint simpleBox(double length, double width);      // 默认：单个矩形
    static Footprint pointMass(double size = 0.02);               // 点质量(倒车/无图)
    static Footprint compound(double body_len, double body_wid,   // 机身 box + 四角圆
                              double leg_r, double leg_dx, double leg_dy);
    static Footprint capsuleBody(double half_len, double radius); // 沿 x 的胶囊机身

    // ---- 从 ROS Float64MultiArray 解析(见 .cpp 的编码说明) ----
    static Footprint fromFloatArray(const std_msgs::msg::Float64MultiArray& msg);

    // 从 /robot_collision_models 的 Float64MultiArray 解析出各档真实体积模型(N=1/4/6/8/10)。
    // 编码: data[0]=档数; 每档 [kind,count,payload]:
    //   kind=1 圆(payload=cx,cy,r); kind=0 多边形(payload=count 个 x,y; base_link 系，已含 margin)。
    // 返回每档一个 Footprint(圆->addCircle, 多边形->addPolygon)，顺序与发布顺序一致。
    static std::vector<Footprint> parseFootprints(const std_msgs::msg::Float64MultiArray& msg);

private:
    std::vector<Shape> shapes_;
};

// 构建机器人静态体积档位(由粗到精)，返回的 vector 顺序固定，
// 索引与 Robot_config::VolumeModel 对齐：
//   [0]质点 [1]粗糙(外接圆) [2]正常(矩形)
// 各档的几何构建细节都集中实现在 Go2_footprint.cpp，便于后续扩展更复杂的模型。
std::vector<Footprint> buildStaticVolumes(double length, double width, double point_mass = 0.02);

}  // namespace go2

#endif  // DYNAMICS_PLANNER_NAV_GO2_FOOTPRINT_HPP

