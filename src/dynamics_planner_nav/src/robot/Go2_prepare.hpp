// Go2_prepare.hpp — Robot_config 的"就绪检查 + 状态机"模块
//
// 把 setup() 每帧用到的就绪判定(checkGazeboPaused / checkMapReady / checkGoalReached)
// 以及速度状态机(updateSpeedStateMachine)从 Go2.cpp 抽到 Go2_prepare.cpp，
// 让 Go2.cpp 更聚焦构造/IO/访问器。
//
// 注意：这些都是 Robot_config 的成员函数，声明仍在 Go2.hpp(类内)；本头文件只作
// 模块说明 + 统一包含，便于 Go2_prepare.cpp 引用。

#ifndef DYNAMICS_PLANNER_NAV_GO2_PREPARE_HPP
#define DYNAMICS_PLANNER_NAV_GO2_PREPARE_HPP

#include "Go2.hpp"

#endif  // DYNAMICS_PLANNER_NAV_GO2_PREPARE_HPP

