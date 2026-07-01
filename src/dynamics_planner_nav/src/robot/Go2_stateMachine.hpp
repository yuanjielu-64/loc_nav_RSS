// Go2_stateMachine.hpp — Robot_config 的"速度状态机"模块
//
// 把速度状态机(updateSpeedStateMachine)从 Go2_prepare.cpp 抽出来单独成文，
// 让 Go2_prepare.cpp 只聚焦"就绪检查"(checkGazeboPaused/checkMapReady/checkGoalReached)，
// 状态切换逻辑(NORMAL/CAUTIOUS/BRAKE/RECOVER 之间的迟滞判定)集中在 Go2_stateMachine.cpp。
//
// 注意：updateSpeedStateMachine 是 Robot_config 的成员函数，声明仍在 Go2.hpp(类内)；
// 本头文件只作模块说明 + 统一包含，便于 Go2_StateMachine.cpp 引用。

#ifndef DYNAMICS_PLANNER_NAV_GO2_STATEMACHINE_HPP
#define DYNAMICS_PLANNER_NAV_GO2_STATEMACHINE_HPP

#include "Go2.hpp"

#endif  // DYNAMICS_PLANNER_NAV_GO2_STATEMACHINE_HPP
