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

    protected:
        void updateRobotState();


        void frontBackParameters(Robot_config &robot);

        void normalParameters(Robot_config &robot);

        void lowSpeedParameters(Robot_config &robot);

        void recoverParameters(Robot_config &robot);

        bool handleNoMapPlanning(geometry_msgs::msg::Twist &cmd_vel);
        bool handleNormalSpeedPlanning(geometry_msgs::msg::Twist &cmd_vel, std::pair<std::vector<PoseState>, bool> &best_traj, double dt);
        bool handleLowSpeedPlanning(geometry_msgs::msg::Twist &cmd_vel, std::pair<std::vector<PoseState>, bool> &best_traj, double dt);

        bool handleAbnormalPlaning(geometry_msgs::msg::Twist &cmd_vel, std::pair<std::vector<PoseState>, bool> &best_traj, double dt);
        void publishCommand(geometry_msgs::msg::Twist &cmd_vel, double linear, double angular) const;

        double calculateDistanceToCarEdge(
        double carX, double carY, double cosTheta, double sinTheta,
        double halfLength, double halfWidth, const std::vector<double>& obs);

        bool ddp_planning(PoseState &state, PoseState &state_odom,
                                 std::pair<std::vector<PoseState>, bool> &best_traj, double dt);

        std::pair<std::vector<PoseState>, std::vector<PoseState> > generateTrajectory(
            PoseState &state, PoseState &state_odom, double angular_velocity);

        std::pair<std::vector<PoseState>, std::vector<PoseState> > generateTrajectory(
            PoseState &state, PoseState &state_odom, std::vector<std::pair<double, double>> &perturbations);

        std::pair<std::vector<PoseState>, std::vector<PoseState> > generateTrajectory(
            PoseState &state, PoseState &state_odom, double v, double w);

        void motion(PoseState &state, double velocity, double angular_velocity, double t) const;

        void process_segment(int thread_id, int start, int end, PoseState &state, PoseState &state_odom, Window &dw,
                                     std::vector<std::pair<double, double>> &pairs,
                                     std::vector<Cost> &thread_costs,
                                     std::vector<std::pair<std::vector<PoseState>, std::vector<PoseState> > > &
                                     thread_trajectories,
                                     std::vector<std::vector<std::pair<double, double>>> &thread_pairs);

        void normalize_costs(std::vector<Cost> &costs);

        Cost evaluate_trajectory(std::pair<std::vector<PoseState>, std::vector<PoseState> > &traj, double &dist,
                                         std::vector<double> &last_position);

        double calc_to_goal_cost(const std::vector<PoseState> &traj);

        double calc_speed_cost(const std::vector<PoseState> &trajs) const;


        double calc_obs_cost(const std::vector<PoseState> &traj, double &t);

        double pointToSegmentDistance(const PoseState& p1, const PoseState& p2, const std::vector<double>& o);

        double calc_ori_cost(const std::vector<PoseState> &traj);

        double calc_path_cost(const std::vector<PoseState> &traj) const;

        Window calc_dynamic_window(PoseState &state, double dt) const;


        std::atomic<bool> timeout_flag{false};

        bool use_goal_cost_ = false;
        bool use_speed_cost_ = false;
        bool use_path_cost_ = false;
        bool use_ori_cost_ = false;
        bool use_space_cost_ = false;

        double robot_radius_ = 0.03;
        double distance = 0.0;

        double current_vel = 0.0;
        int num_threads{};
        double obs_range_ = 4;

        int nr_pairs_ = 20;
        int nr_steps_ = 20;
        double linear_stddev = 0.05;
        double angular_stddev = 0.05;

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

        double delta_v_sum = FLT_MIN;
        double delta_w_sum = FLT_MIN;

        double n{};
    };


}

#endif
