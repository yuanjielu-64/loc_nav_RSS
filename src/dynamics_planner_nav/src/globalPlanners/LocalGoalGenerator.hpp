// LocalGoalGenerator.hpp — 状态相关的 local goal(前瞻点)生成/获取
//
// 注意：这不是一个 planner(不做轨迹搜索)，只负责【如何从全局路径上取/生成 local goal】。
//
// 设计动机(契合 Decremental Dynamics Planning 范式)：
//   global 路径(/plan)是 point-mass 无动力学的；local 规划(DDP)是全动力学的。
//   二者的 handoff 点 = local goal。固定欧氏前瞻距离与动力学无关，是 paradigm 内
//   的逻辑缺口。这里把 local goal 的"前瞻距离"做成【由当前状态/动力学可达性推出】：
//     - NORMAL  ：可达边界拉远 → 轨迹平滑、跟住大方向(开阔高速)。
//     - CAUTIOUS：拉近 → 贴线、反应灵敏、窄道平滑。
//     - RECOVER ：另一套保守近点 → 温和重新贴回全局路径。
//   选点机制统一用【沿路径弧长】(先把机器人投影到路径最近点，再往前累计弧长)，
//   而非"离机器人欧氏 ≥ 阈值的第一个点"，从而修掉 U 形回绕时选到回程点的 bug。
//
// 模块是【纯函数】：只吃 odom 帧路径点 + 一个查询结构，返回被选中点的下标；
// 不持有 ROS、不加锁、不写 Robot_config，便于单测与复用。

#ifndef DYNAMICS_PLANNER_NAV_LOCAL_GOAL_GENERATOR_HPP
#define DYNAMICS_PLANNER_NAV_LOCAL_GOAL_GENERATOR_HPP

#include <vector>

namespace lgoal {

// local goal 选取的输入快照(全部 odom 帧 / 当前状态量)。
struct LocalGoalQuery {
    int    robot_state   = 1;     // Robot_config::RobotState 的枚举值(1=NORMAL...)
    double rx            = 0.0;    // 机器人 odom 位置 x
    double ry            = 0.0;    // 机器人 odom 位置 y
    double base_lookahead = 1.0;   // 标称前瞻距离(local_goal_distance)，作各状态的基准/地板
    double max_vel_x     = 1.0;    // 当前状态前向速度上限(已随状态缩放)
    double horizon_time  = 1.0;    // 规划时域 Σdt(s)；<=0 时用内部兜底
};

// ===== 三套"前瞻距离方程"(每个状态一套) =====
// 只决定"看多远"(米)，不决定"看哪个点"；后者由 generateLocalGoal 的弧长机制统一完成。
double lookaheadNormal  (const LocalGoalQuery &q);   // 拉远：动力学可达边界
double lookaheadCautious(const LocalGoalQuery &q);   // 拉近：窄道平滑
double lookaheadRecover (const LocalGoalQuery &q);   // 脱困：保守近点

// 统一入口：按 robot_state 选对应方程算出前瞻距离 L，再沿 path_odom 的弧长挑点。
// 返回被选为 local goal 的下标；path_odom 为空返回 -1。
int generateLocalGoal(const std::vector<std::vector<double>> &path_odom,
                      const LocalGoalQuery &q);

}  // namespace lgoal

#endif  // DYNAMICS_PLANNER_NAV_LOCAL_GOAL_GENERATOR_HPP
