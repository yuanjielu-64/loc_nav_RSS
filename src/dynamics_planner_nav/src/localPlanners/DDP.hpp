/*
 * Copyright (C) 2018 Erion Plaku
 * All Rights Reserved
 *
 *       Created by Erion Plaku
 *       Computational Robotics Group
 *       Department of Electrical Engineering and Computer Science
 *       Catholic University of America
 *
 *       www.robotmotionplanning.org
 *
 * Code should not be distributed or used without written permission from the
 * copyright holder.
 */

#ifndef Antipatrea__DDP_HPP_
#define Antipatrea__DDP_HPP_

#include "../robot/Go2.hpp"
#include "localPlanners/LocalPlannerBase.hpp"
#include "utils/Algebra.hpp"
#include <numeric>
#include <thread>
#include <mutex>
#include <chrono>
#include <atomic>
#include <random>
#include <future>
#include <geometry_msgs/msg/twist.hpp>

namespace Antipatrea {
    using PoseState = Robot_config::PoseState;

    // 继承公共基类：normalizeAngle / updateVelocity / calculateTheta 等通用纯函数
    // 由 LocalPlannerBase 提供，DDP 不再自带副本。
    class DDP : public LocalPlannerBase {
    public:
        DDP() = default;

        ~DDP() = default;

        bool Solve(int nrIters, double dt, bool &canBeSolved);


        Robot_config *robot = nullptr;

        // 全向控制量(Go2)：vx 机体前向、vy 机体侧向、w 偏航角速度。
        // 取代旧的 std::pair<double,double>(只有 v,w)，让采样/轨迹支持横移。
        struct Control {
            double vx{0.0};
            double vy{0.0};
            double w{0.0};
        };

    protected:
        void updateRobotState();


        void frontBackParameters(Robot_config &robot);

        void normalParameters(Robot_config &robot);

        void lowSpeedParameters(Robot_config &robot);

        void recoverParameters(Robot_config &robot);

        bool handleNoMapPlanning(geometry_msgs::msg::Twist &cmd_vel);
        bool handleNormalSpeedPlanning(geometry_msgs::msg::Twist &cmd_vel, std::pair<std::vector<PoseState>, bool> &best_traj, double dt);
        bool handleLowSpeedPlanning(geometry_msgs::msg::Twist &cmd_vel, std::pair<std::vector<PoseState>, bool> &best_traj, double dt);

        bool handleAbnormalPlanning(geometry_msgs::msg::Twist &cmd_vel, std::pair<std::vector<PoseState>, bool> &best_traj, double dt);
        // 斥力(RECOVER ①)三态结果：
        //   Safe   = 8 方向都 ≥ kPushDist(没贴障)，跳过斥力，继续 ②③④；
        //   Pushed = 有方向贴障且还推得动，已发顶出指令，调用方 return；
        //   Stuck  = 有方向贴障但推不动(hard_mag 不再创新低)，交给旋转转一步(推不动计数保留，不重数)。
        enum class PushResult { Safe, Pushed, Stuck };
        PushResult repulsionPush(geometry_msgs::msg::Twist &cmd_vel);
        // 旋转(RECOVER ②④)：朝 local goal 方向原地转身。目标在左→左转(+w)、右→右转(-w)；
        //   带最小转速防停滞；无 local goal 时默认左转扫描。原地转不前进 → 清零成功计数。
        void rotateToGoal(geometry_msgs::msg::Twist &cmd_vel,
                          double add_vx = 0.0, double add_vy = 0.0);
        // RECOVER 进度看门狗 + 逃逸：实测位移长时间不增(ddp恒有解但狗不动)→ 倒退+朝更空一侧转身脱困。
        //   返回 true=本帧已被逃逸动作接管(调用方 return)；false=未卡死，交还 Rotate/Plan。
        bool recoverEscape(geometry_msgs::msg::Twist &cmd_vel);
        // RECOVER 子阶段(存于 robot->recover_phase)：Rotate=朝目标对准，Plan=规划(锁存，不再判朝向)。
        static constexpr int kRecoverRotate = 0;
        static constexpr int kRecoverPlan   = 1;
        // publishCommand 已统一到 LocalPlannerBase(static)，调用方式：publishCommand(*robot, cmd_vel, ...)。

        bool ddp_planning(PoseState &state, PoseState &state_odom,
                                 std::pair<std::vector<PoseState>, bool> &best_traj, double dt);

        std::pair<std::vector<PoseState>, std::vector<PoseState> > generateTrajectory(
            PoseState &state, PoseState &state_odom, double angular_velocity);

        std::pair<std::vector<PoseState>, std::vector<PoseState> > generateTrajectory(
            PoseState &state, PoseState &state_odom, std::vector<Control> &perturbations);

        std::pair<std::vector<PoseState>, std::vector<PoseState> > generateTrajectory(
            PoseState &state, PoseState &state_odom, double vx, double vy, double w);

        // 全向运动模型：vx/vy 为机体系前/侧向速度，w 偏航角速度。
        void motion(PoseState &state, double vx, double vy, double angular_velocity, double t) const;

        void process_segment(int thread_id, int start, int end, PoseState &state, PoseState &state_odom, Window &dw,
                                     std::vector<Control> &pairs,
                                     std::vector<Cost> &thread_costs,
                                     std::vector<std::vector<Control>> &thread_pairs,
                                     int &processed_count);

        void normalize_costs(std::vector<Cost> &costs);

        Cost evaluate_trajectory(std::pair<std::vector<PoseState>, std::vector<PoseState> > &traj, double &dist,
                                         std::vector<double> &last_position);

        double calc_to_goal_cost(const std::vector<PoseState> &traj);

        double calc_speed_cost(const std::vector<PoseState> &trajs) const;


        // space_cost_out 回传：轨迹后 1/4 段前向锥内障碍的逼近量(obs_range_-min_front_dist 累加)，
        // 与函数返回的障碍代价不是一回事；返回值=obs_cost，引用出参=space_cost。
        double calc_obs_cost(const std::vector<PoseState> &traj, double &space_cost_out);


        double calc_ori_cost(const std::vector<PoseState> &traj);

        double calc_path_cost(const std::vector<PoseState> &traj) const;


        std::atomic<bool> timeout_flag{false};
        // 单帧并行评估的时间预算(ms)：超过即令各线程提前停止，用已算出的候选选解，
        // 保证主循环维持 20Hz(50ms 预算)。solve_deadline_ 是本帧评估的截止时刻。
        double solve_time_budget_ms_ = 40.0;
        std::chrono::high_resolution_clock::time_point solve_deadline_;
        // 诊断：本帧候选被淘汰的两类原因计数(并行段前清零，各线程原子累加)。
        // collision = 轨迹撞障碍(obs_cost=1e6)；too_short = 轨迹太短被判废(path_cost=1e6)。
        std::atomic<int> reject_collision_{0};
        std::atomic<int> reject_too_short_{0};

        bool use_goal_cost_ = false;
        bool use_speed_cost_ = false;
        bool use_path_cost_ = false;
        bool use_ori_cost_ = false;
        bool use_space_cost_ = false;
        // 是否允许侧移采样(全向 vy)。按状态切换：NORMAL 关(高速直行+转向更稳)，
        // LOW_SPEED 开(低速窄道可侧身)，RECOVER 暂关。关闭时所有候选 vy≡0，退化为差速。
        bool use_vy_ = false;

        double robot_radius_ = -0.01;
        // 候选轨迹被接受所需的【最小行进距离】(整条轨迹的总弧长下限)。
        // 轨迹总弧长 <= 该值则判无效(过滤"几乎不动"的候选，强制最小前进量)。原名 distance。
        double min_traj_length_ = 0.0;

        // 使用哪一档外包碰撞模型做碰撞: 索引对应 /robot_collision_models 的 N=[1,4,6,8,10]。
        // 2 = N=6(六边形, 比 N=4 矩形更贴合狗轮廓, valid 更高、避障更准)。
        // Release 构建下 800 候选仍 <70ms，精度可负担。收不到模型时回退盒子 footprint。
        int volume_index_ = 2;

        // 一次 solve 内缓存选中的真实外包模型(go2::Footprint)，供 8 线程只读共享，
        // 避免每候选 calc_obs_cost 重复 getDynamicVolumes() 拷贝。
        // 上游(发布端)保证收到时非空，setup() 也已用 checkVolumeReady() 卡就绪。
        go2::Footprint solve_volume_;
        double solve_volume_circ_r_ = 0.30;   // 该模型外接圆半径(远场粗筛用)

        // 本帧障碍的【扁平连续数组】(cache 友好)，由 ddp_planning 一次性准备：
        //  - 取一次 getDataMap()，避免每条候选轨迹深拷贝整个障碍 vector<vector<double>>；
        //  - 同时做 per-solve 可达预筛，剔除"任何轨迹点都够不到"的远障碍。
        // 8 线程只读共享，calc_obs_cost 直接遍历它，不再调用 getDataMap()。
        std::vector<float> solve_obs_x_;
        std::vector<float> solve_obs_y_;

        double current_vel = 0.0;
        int num_threads{};
        double obs_range_ = 4;

        // 障碍代价整形(归一化势场)，可按模式在 Parameters.cpp 覆盖。代价函数：
        //   md <= obs_lethal_dist_     -> obs_lethal_cost_  (峰值)
        //   md >= obs_influence_dist_  -> 0
        //   否则 obs_lethal_cost_ * ((1/md - 1/infl)/(1/lethal - 1/infl))^obs_shape_p_
        double obs_lethal_dist_ = 0.15;     // 到此距离代价达峰值
        double obs_lethal_cost_ = 300.0;    // 峰值代价
        double obs_influence_dist_ = 1.0;   // 影响半径(多远开始 >0)：≥此距离零惩罚
        double obs_shape_p_ = 2.0;          // 陡度指数(>1 更集中在近处)

        int nr_pairs_ = 20;
        int nr_steps_ = 20;
        double linear_stddev = 0.05;
        double angular_stddev = 0.05;
        double lateral_stddev = 0.05;   // 全向侧移采样标准差(vy)

        int v_steps_ = 20;
        int w_steps_ = 20;

        PoseState parent;
        PoseState parent_odom;

        std::vector<double> timeInterval;

        double to_goal_cost_gain_ = 0.8;
        double obs_cost_gain_ = 0.5;
        double speed_cost_gain_ = 0.4;
        double path_cost_gain_ = 0.4;
        double ori_cost_gain_ = 0.3;
        double aw_cost_gain_ = 0.2;
        double space_cost_gain_ = 0.5;

        // 朝向代价整形指数：calc_ori_cost 累加 |夹角|^ori_shape_p_。
        //   =1 线性(角度越大惩罚越大，等比)；>1 超线性(大偏离惩罚被放大，强逼车头转正)。
        //   NORMAL/CAUTIOUS 用 1.0(温和)，RECOVER 用 2.0(朝向主导，狠罚背向目标)。
        double ori_shape_p_ = 1.0;

        double delta_v_sum = FLT_MIN;
        double delta_w_sum = FLT_MIN;
        double delta_vy_sum = 0.0;   // 全向侧移加权解(vy)

        double n{};
    };


}

#endif
