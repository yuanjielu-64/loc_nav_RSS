// Go2_parameters.cpp — Robot_config 的【规划器参数】读出/写入(parameter-learning 接口)
//
// 把整组 planner 可调参数(go2::PlannerParams)一次性读出(getTuningParams)或写入
// (setTuningParams)。从 Go2.cpp 抽到此处，让参数面集中、Go2.cpp 更聚焦于节点/状态逻辑。
// 参数容器定义见 Go2_parameters.hpp。

#include "Go2.hpp"

Robot_config::TuningParams Robot_config::getTuningParams() const {
    TuningParams params{};
    params.max_vel_x = max_vel_x;
    params.max_vel_y = max_vel_y;
    params.max_vel_theta = max_vel_theta;
    params.vx_sample = static_cast<int>(vx_sample);
    params.vTheta_samples = static_cast<int>(vTheta_samples);
    params.path_distance_bias = path_distance_bias;
    params.goal_distance_bias = goal_distance_bias;
    params.nr_pairs_ = static_cast<int>(nr_pairs_);
    params.nr_steps_ = static_cast<int>(nr_steps_);
    params.linear_stddev = linear_stddev;
    params.angular_stddev = angular_stddev;
    params.lambda = lambda;
    params.local_goal_distance = local_goal_distance;
    params.distance = distance;
    params.robot_radius_ = robot_radius_;
    params.dt = dt;
    return params;
}

void Robot_config::setTuningParams(const TuningParams &tp) {
    max_vel_x = tp.max_vel_x;
    max_vel_y = tp.max_vel_y;
    max_vel_theta = tp.max_vel_theta;
    vx_sample = tp.vx_sample;
    vTheta_samples = tp.vTheta_samples;
    path_distance_bias = tp.path_distance_bias;
    goal_distance_bias = tp.goal_distance_bias;
    nr_pairs_ = tp.nr_pairs_;
    nr_steps_ = tp.nr_steps_;
    linear_stddev = tp.linear_stddev;
    angular_stddev = tp.angular_stddev;
    lambda = tp.lambda;
    local_goal_distance = tp.local_goal_distance;
    distance = tp.distance;
    robot_radius_ = tp.robot_radius_;
    dt = tp.dt;
    tuning_params_ = tp;
}

