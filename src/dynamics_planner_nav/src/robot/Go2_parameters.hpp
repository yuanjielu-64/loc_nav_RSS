// Go2_parameters.hpp — Go2 导航的【规划器可调参数】定义
//
// 这里集中放 planner 的可调参数容器 PlannerParams。它就是未来 parameter learning 的
// 【学习接口/参数面】：要被学习/搜索/序列化的旋钮都收在这一个结构体里，和 Robot_config
// 的 ROS 接线、运行时状态分开，便于单独读写一整组参数(getTuningParams/setTuningParams)。
//
// 历史：原为 Robot_config 内嵌的 TuningParams，现抽到独立文件。Robot_config 仍用
//       using TuningParams = go2::PlannerParams; 做别名，老代码 Robot_config::TuningParams
//       / getTuningParams() 不受影响。

#ifndef DYNAMICS_PLANNER_NAV_GO2_PARAMETERS_HPP
#define DYNAMICS_PLANNER_NAV_GO2_PARAMETERS_HPP

namespace go2 {

// 规划器可调参数快照(parameter-learning 面)：一整组旋钮，可整体读出/写入。
struct PlannerParams {
    double max_vel_x;            // 前向速度上限(m/s)
    double max_vel_y;            // 侧移速度上限(m/s)
    double max_vel_theta;        // 偏航角速度上限(rad/s)
    int    vx_sample;            // 前向速度采样数
    int    vTheta_samples;       // 角速度采样数
    double path_distance_bias;   // 路径贴合代价权重
    double goal_distance_bias;   // 目标趋近代价权重
    int    nr_pairs_;            // 采样候选轨迹条数
    int    nr_steps_;            // 每条轨迹步数(须与 timeInterval 长度一致)
    double linear_stddev;        // 线速度高斯扰动标准差
    double angular_stddev;       // 角速度高斯扰动标准差
    double lambda;               // 软加权温度(MPPI/DDP 选解)
    double local_goal_distance;  // 前瞻局部目标距离(m)
    double distance;             // 通用距离阈值
    double robot_radius_;        // 机器人半径(避障用)
    double dt;                   // 规划步长(s)
};

}  // namespace go2

#endif  // DYNAMICS_PLANNER_NAV_GO2_PARAMETERS_HPP

