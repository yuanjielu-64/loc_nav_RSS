// Go2_stateMachine.cpp — 速度状态机的实现(声明见 Go2.hpp)。
#include "Go2_stateMachine.hpp"

//==============================================================================
// 速度状态机：用【实测线速度 vs 期望速度】+ 时间迟滞自动切 NORMAL/CAUTIOUS/BRAKE。
// 由 velocityCallback 每帧调用(传入 |linear.x|)。仅对 DDP 系生效。
//   NORMAL  ：实测长期 < HIGH 阈值(跑不快=进窄道) 持续 0.5s → 降 CAUTIOUS。
//   CAUTIOUS：实测 ≥ LOW 阈值(+迟滞) 持续 0.5s → 回 NORMAL；近停 0.5s → 切 BRAKE。
//   其它(RECOVER…)：只清计时。
// 阈值随 max_vel_x(当前状态前向上限)缩放，迟滞用 *_time/*_active 成员防抖。
//==============================================================================
void Robot_config::updateSpeedStateMachine(double linear_speed) {
    // 仅 DDP 系(DWA/MPPI 系自带速度采样，不走这套)。
    if (algorithm == DWA || algorithm == DWA_DDP ||
        algorithm == MPPI || algorithm == MPPI_DDP)
        return;

    // 没有有效导航目标时，狗本就静止——"低速/近停"是常态而非卡住，绝不能据此切
    // CAUTIOUS/BRAKE(否则开机还没点目标就自己刹停)。仅在"已收到目标且未到达"时才运行；
    // 其余情况清空迟滞计时，保持当前状态不动。
    if (!global_goal_received || global_goal_reached) {
        normal_to_low_active = false;
        low_to_normal_active = false;
        low_to_brake_active  = false;
        // 关键：空闲期(INIT/IDLE)也要跟踪状态，否则到达目标后状态切 IDLE 却没被记录，
        // last_speed_machine_state_ 残留 NORMAL；下次发目标重回 NORMAL 时 just_entered_normal
        // 误判为 false → 起步宽限不重置 → 新目标刚起步就被当低速切 CAUTIOUS。
        last_speed_machine_state_ = getRobotState();
        return;
    }

    // 阈值基于【桥速度(常量)】，不随状态变——只有这样 T_回 > T_降 才能形成稳定迟滞带，
    // 避免 NORMAL↔CAUTIOUS 横跳。且 T_回 必须 < CAUTIOUS 天花板(bridge*0.5)否则被限速
    // 封顶、永远回不去。带内[T_降, T_回]保持当前状态。
    const double T_down = bridge_max_velocity_ * 0.25;   // < 此速度 → 降 CAUTIOUS
    const double T_up   = bridge_max_velocity_ * 0.3;    // ≥ 此速度 → 回 NORMAL
    // 近停阈值按桥速缩放(不能用固定常量)：否则当桥速很小(如 0.1)时，实测速度永远小于固定
    // 阈值 → "近停"恒真 → 一直误判卡住切 BRAKE/RECOVER。bridge×0.1 在 bridge=1.0 时为 0.1。
    const double NEAR_STOP = bridge_max_velocity_ * 0.1;
    const double NEAR_STOP_HOLD = 2.0;   // 近停需持续这么久才切 BRAKE(s)
    const double HOLD            = 1.5;   // 迟滞：连续满足多久才切(s)(用于 CAUTIOUS→NORMAL 回升)
    const double NORMAL_TO_LOW_HOLD = 1.5;   // NORMAL→CAUTIOUS 降速：需持续低速这么久才切(s)
    const double NORMAL_GRACE    = 1.0;   // 进入 NORMAL 后的起步加速宽限(s)：期内不判降速

    const auto t_now = now();

    // 检测"刚进入 NORMAL"：进入时必须复位迟滞计时(normal_to_low_active=false)并记录进入
    // 时刻，否则会用上一轮残留的旧 normal_to_low_time，第一帧就 (t_now-旧时间)>=HOLD 而
    // 误切 CAUTIOUS(即"一帧掉")。同时起步零速属正常加速过程，宽限期内不计入低速。
    const RobotState cur_state = getRobotState();
    const bool just_entered_normal = (cur_state == NORMAL && last_speed_machine_state_ != NORMAL);
    last_speed_machine_state_ = cur_state;
    if (just_entered_normal) {
        normal_to_low_active = false;
        normal_enter_time = t_now;
    }

    if (getRobotState() == NORMAL) {
        low_to_normal_active = false;
        low_to_brake_active  = false;

        // 起步加速宽限期内：不判降速(狗刚从静止起步，实测速度必然低，不是窄道卡住)。
        const bool in_grace = (t_now - normal_enter_time).seconds() < NORMAL_GRACE;

        // 在 NORMAL 却长期跑不快 → 进窄道，降 CAUTIOUS。
        if (!in_grace && linear_speed < T_down) {
            if (!normal_to_low_active) {
                normal_to_low_time = t_now;
                normal_to_low_active = true;
            } else if ((t_now - normal_to_low_time).seconds() >= NORMAL_TO_LOW_HOLD) {
                RCLCPP_INFO(get_logger(), "持续低速 1s → 切 CAUTIOUS");
                setRobotState(CAUTIOUS);
                normal_to_low_active = false;
            }
        } else {
            normal_to_low_active = false;
        }

    } else if (getRobotState() == CAUTIOUS) {
        normal_to_low_active = false;

        // 又能跑快 → 回 NORMAL。
        if (linear_speed >= T_up) {
            if (!low_to_normal_active) {
                low_to_normal_time = t_now;
                low_to_normal_active = true;
            } else if ((t_now - low_to_normal_time).seconds() >= HOLD) {
                RCLCPP_INFO(get_logger(), "恢复高速 0.5s → 回 NORMAL");
                setRobotState(NORMAL);
                low_to_normal_active = false;
            }
        } else {
            low_to_normal_active = false;
        }

        // 几乎停住 → 卡了，切 BRAKE。BRAKE 在 DDP::handleAbnormalPlanning 里刷停后转入 RECOVER。
        if (linear_speed < NEAR_STOP) {
            if (!low_to_brake_active) {
                low_to_brake_time = t_now;
                low_to_brake_active = true;
            } else if ((t_now - low_to_brake_time).seconds() > NEAR_STOP_HOLD) {
                RCLCPP_INFO(get_logger(), "低速近停 → 切 BRAKE");
                setRobotState(BRAKE);
                low_to_brake_active = false;
            }
        } else {
            low_to_brake_active = false;
        }

    } else {   // RECOVER 等：清 NORMAL/CAUTIOUS 的计时
        normal_to_low_active = false;
        low_to_normal_active = false;
        low_to_brake_active  = false;

        // RECOVER → CAUTIOUS：实测 |vx| ≥ 0.2×bridge 持续 2s = 狗真的跑起来了(脱困成功) → 退回 CAUTIOUS。
        //   不再用"ddp 有解 5 帧"那种廉价退出(recoverParameters 下 ddp 几乎恒有解，会 0.25s 就溜回 LOW)。
        if (getRobotState() == RECOVER) {
            const double RECOVER_EXIT_SPEED = bridge_max_velocity_ * 0.1;   // 退出速度阈值(RECOVER 前进上限仅 0.25×bridge，故取 0.1 才够得到)
            const double RECOVER_EXIT_HOLD  = 1.0;                          // 需持续这么久(s)
            if (linear_speed >= RECOVER_EXIT_SPEED) {
                if (!recover_fast_active) {
                    recover_fast_time = t_now;
                    recover_fast_active = true;
                } else if ((t_now - recover_fast_time).seconds() >= RECOVER_EXIT_HOLD) {
                    RCLCPP_INFO(get_logger(), "RECOVER 持续提速 2s → 切 CAUTIOUS");
                    recover_exit_time = t_now;   // 记录退出时刻：供 BRAKE 的复发升级(recover_level++)判定
                    setRobotState(CAUTIOUS);
                    recover_fast_active = false;
                }
            } else {
                recover_fast_active = false;   // 一旦慢下来就重新计时
            }
        } else {
            recover_fast_active = false;
        }
    }
}
