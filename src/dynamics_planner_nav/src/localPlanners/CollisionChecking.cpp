// CollisionChecking.cpp — 局部规划器共享碰撞检测原语【实现】
//
// 目前热路径原语(boxEdgeDistance)是 header inline，定义在 CollisionChecking.hpp。
// 本文件预留给将来较重/非热路径的碰撞例程(例如多边形 footprint 的精确碰撞、
// 扫掠体碰撞、与 costmap 的批量查询等)，以及后续可能抽出的 planner 碰撞基类实现。

#include "localPlanners/CollisionChecking.hpp"

namespace Antipatrea {
namespace collision {

// (此处暂无非内联实现；新增较重的碰撞例程时在这里定义并在 .hpp 中声明。)

}  // namespace collision
}  // namespace Antipatrea

