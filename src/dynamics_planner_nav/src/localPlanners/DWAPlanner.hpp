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

#ifndef Antipatrea__DWAPlanner_HPP_
#define Antipatrea__DWAPlanner_HPP_

#include "../robot/Go2.hpp"
#include "localPlanners/LocalPlannerBase.hpp"
#include "utils/Algebra.hpp"
#include <numeric>
#include <thread>
#include <mutex>
#include <chrono>
#include "utils/Timer.hpp"
#include <geometry_msgs/msg/twist.hpp>



namespace Antipatrea {
    using PoseState = Robot_config::PoseState;

    // 继承公共基类：通用纯函数(normalizeAngle/calculateTheta 等)由基类提供。
    class DWAPlanner : public LocalPlannerBase {
    public:
        DWAPlanner() = default;

        virtual ~DWAPlanner() = default;

        virtual bool Solve(int nrIters, double tmax, bool &canBeSolved);

        Robot_config *robot = nullptr;

    protected:
        void updateRobotState();

        virtual void frontBackParameters(Robot_config &robot);

        virtual void normalParameters(Robot_config &robot);

        virtual void lowSpeedParameters(Robot_config &robot);

        virtual void recoverParameters(Robot_config &robot);

        virtual bool handleNoMapPlanning(geometry_msgs::msg::Twist &cmd_vel);

        virtual bool handleNormalSpeedPlanning(geometry_msgs::msg::Twist &cmd_vel,
                                               std::pair<std::vector<PoseState>, bool> &best_traj, double dt);

        virtual bool handleLowSpeedPlanning(geometry_msgs::msg::Twist &cmd_vel,
                                            std::pair<std::vector<PoseState>, bool> &best_traj, double dt);

        virtual bool handleAbnormalPlanning(geometry_msgs::msg::Twist &cmd_vel,
                                           std::pair<std::vector<PoseState>, bool> &best_traj, double dt);

        virtual void publishCommand(geometry_msgs::msg::Twist &cmd_vel, double linear, double angular);


        virtual double recover(PoseState &state, PoseState &state_odom,
                               std::pair<std::vector<PoseState>, bool> &best_traj, bool &results);

        virtual bool dwa_planning(PoseState &state, PoseState &state_odom,
                                  std::pair<std::vector<PoseState>, bool> &best_traj, double time);

        virtual double calculateDistanceToCarEdge(
            double carX, double carY, double cosTheta, double sinTheta,
            double halfLength, double halfWidth, const std::vector<double> &obs);

        virtual std::pair<std::vector<PoseState>, std::vector<PoseState> > generateTrajectory(
            PoseState &state, PoseState &state_odom, double angular_velocity);

        virtual std::pair<std::vector<PoseState>, std::vector<PoseState> > generateTrajectory(
            PoseState &state, PoseState &state_odom, double v, double w);

        virtual void motion(PoseState &state, double velocity, double angular_velocity);

        virtual void normalize_costs(std::vector<Cost> &costs);

        virtual void process_segment(int thread_id, int start, int end, PoseState &state, PoseState &state_odom,
                                     double velocity_resolution,
                                     double angularVelocity_resolution, Window &dw,
                                     std::vector<Cost> &thread_costs,
                                     std::vector<std::pair<std::vector<PoseState>, std::vector<PoseState> > > &
                                     thread_trajectories);

        virtual double calc_to_goal_cost(const std::vector<PoseState> &traj);

        virtual double calc_speed_cost(const std::vector<PoseState> &traj);

        virtual Cost evaluate_trajectory(std::pair<std::vector<PoseState>, std::vector<PoseState> > &traj, double &dist,
                                         std::vector<double> &last_position);

        virtual Cost evaluate_trajectory(std::vector<PoseState> &traj, double &dist,
                                         std::vector<double> &last_position);

        virtual double calc_obs_cost(const std::vector<PoseState> &traj);

        virtual double calc_ori_cost(const std::vector<PoseState> &traj);


        virtual double calc_path_cost(const std::vector<PoseState> &traj);

        virtual double calc_dist_to_path(const std::vector<double> &state);

        virtual Window calc_dynamic_window(PoseState &state);


        bool use_goal_cost_ = false;
        bool use_speed_cost_ = false;
        bool use_path_cost_ = false;
        bool use_ori_cost_ = false;

        double angle_to_goal_ = M_PI / 2;

        double robot_radius_ = 0.03;
        // 候选轨迹被接受所需的最小行进距离(总弧长下限)；轨迹太短则判无效。原名 distance。
        double min_traj_length_ = 0.0;

        int num_threads;
        double obs_range_ = 4;
        int nr_steps_ = 20;
        int v_steps_ = 20;
        int w_steps_ = 20;

        double path_distance_bias = 0.7;
        double goal_distance_bias = 0.8;

        PoseState parent;
        PoseState parent_odom;

        std::vector<double> timeInterval;

        double to_goal_cost_gain_ = 0.8;
        double obs_cost_gain_ = 0.5;
        double speed_cost_gain_ = 0.4;
        double path_cost_gain_ = 0.4;
        double ori_cost_gain_ = 0.3;
        double aw_cost_gain_ = 0.2;

        double n;

        // mtx was unused
        std::vector<std::vector<double> > global_paths;
    };


}

#endif
