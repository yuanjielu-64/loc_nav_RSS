// collisionChecking.hpp — 局部规划器(local planners)共享的碰撞检测原语【声明/内联】
//
// 这里集中放各 planner(DDP/DWA/MPPI/DWA_DDP/MPPI_DDP...) 公用的碰撞/距离计算，
// 让它们不再各自复制一份。属于 planner 层的关注点，故放在 localPlanners/ 下，
// 与 robot/Go2_footprint(机器人物理体积模型) 区分开。
//
// 热路径函数(每次 solve 调用数百万次)定义为 inline，放在头文件里以便编译器内联，
// 没有跨编译单元的调用开销 —— 既共享一份代码、又和原来写在 planner 里同样快。
// 非热路径/较重的碰撞例程实现在 collisionChecking.cpp。

#ifndef ANTIPATREA_COLLISION_CHECKING_HPP_
#define ANTIPATREA_COLLISION_CHECKING_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

namespace Antipatrea {
namespace collision {

// 盒子边缘距离：把障碍点变换到以(carX,carY)为中心、按(cosTheta,sinTheta)旋转的
// 机器人盒子局部系，返回障碍点到盒边的欧氏距离。
//   - 盒外：返回 >=0 的距离
//   - 盒内：夹为 0(不返回负值)
// 这是各 planner 历史碰撞代价沿用的约定(等价于原 calculateDistanceToCarEdge)。
// (carX,carY)=机器人位置；(cosTheta,sinTheta)=调用方预算好的朝向三角值；
// halfLength/halfWidth=机器人盒子半长/半宽；obs=障碍点[x,y]。
inline double boxEdgeDistance(double carX, double carY, double cosTheta, double sinTheta,
                              double halfLength, double halfWidth, const std::vector<double>& obs) {
    const double relX = obs[0] - carX;
    const double relY = obs[1] - carY;

    const double localX = relX * cosTheta + relY * sinTheta;
    const double localY = -relX * sinTheta + relY * cosTheta;

    const double dx = std::max(std::abs(localX) - halfLength, 0.0);
    const double dy = std::max(std::abs(localY) - halfWidth, 0.0);

    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace collision
}  // namespace Antipatrea

#endif  // ANTIPATREA_COLLISION_CHECKING_HPP_

