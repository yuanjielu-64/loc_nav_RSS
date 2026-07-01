// Go2_prepare.cpp — 就绪检查的实现(声明见 Go2.hpp)。
// 速度状态机(updateSpeedStateMachine)已抽到 Go2_stateMachine.cpp。
#include "Go2_prepare.hpp"
#include <cmath>

//==============================================================================
// 就绪检查(setup 每帧调用)
//==============================================================================

bool Robot_config::checkGazeboPaused() const {
    std_msgs::msg::String state_msg;

    // In ROS2, check parameter differently
    bool is_paused = false;
    // Note: In ROS2, you would typically use a parameter or service
    // For now, assume not paused
    if (is_paused) {
        state_msg.data = "PAUSED";
        robot_state_pub_->publish(state_msg);
        return true;
    }

    publishRobotState();
    return false;
}

bool Robot_config::checkMapReady(bool goal_ok) {
    map.clear();

    switch (getRobotState()) {
        // 示例：若 BACK(倒车) 想用 costmap，把下面两行放进 case BACK 即可。
        // case BACK:
        //     map = costmapData;
        //     currentMap = ONLY_COSTMAP_RECEIVED;
        //     break;
        default:
            map = getLaserData();
            currentMap = ONLY_LASER_RECEIVED;
            break;
    }

    if (map.empty() && goal_ok) {
        currentMap = NO_ANY_RECEIVED;
        setRobotState(BLIND);
        return false;
    }

    // 之前因无数据卡在 BLIND，数据回来了恢复 NORMAL，避免永久卡死不避障。
    if (getRobotState() == BLIND) {
        setRobotState(NORMAL);
    }

    return true;
}

bool Robot_config::checkGoalReached(double cur_x, double cur_y) {
    // goalCallback 在 path_goal_mutex_ 下写 global_goal_odom / global_goal_reached，
    // 这里读同一组字段必须同锁，否则 vector::operator= 中途被读会 UB。
    std::lock_guard<std::mutex> lock(path_goal_mutex_);

    if (global_goal_reached) return false;
    if (global_goal_odom.size() < 2 || local_goal_odom.size() < 2) return false;   // 还没收到 goal

    const double dx = global_goal_odom[0] - cur_x;
    const double dy = global_goal_odom[1] - cur_y;
    const double thr_sq = goal_reached_threshold * goal_reached_threshold;
    if (dx*dx + dy*dy > thr_sq) return true;        // 还没到，继续规划

    RCLCPP_INFO(get_logger(),
        "到达 global goal (距离 %.2fm <= %.2fm)，停止规划，等待新目标。",
        std::sqrt(dx*dx + dy*dy), goal_reached_threshold);
    global_goal_reached = true;
    setRobotState(IDLE);
    return false;
}

