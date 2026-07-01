//
// Created by yuanjielu on 12/10/24.
//

#include "localPlanners/MPPI_DDPPlanner.hpp"
#include "localPlanners/DWA_DDPPlanner.hpp"
#include "localPlanners/DWAPlanner.hpp"
#include "localPlanners/MPPIPlanner.hpp"
#include "localPlanners/DDP.hpp"

namespace Antipatrea {

    void DDP::normalParameters(Robot_config &robot) {
        v_steps_ = 20;
        w_steps_ = 20;
        nr_steps_ = (int) robot.timeInterval.size();   // 步数随 timeInterval(单一来源)自适应
        nr_pairs_ = 1500;   // 轨迹数(探索性)：保持多；提速靠优化 calc_obs_cost 而非砍候选

        min_traj_length_ = 0.5;
        robot_radius_ = 0.00;

        use_goal_cost_ = true;
        use_angular_cost_ = true;
        use_path_cost_ = true;
        use_speed_cost_ = false;
        use_ori_cost_ = true;
        use_space_cost_ = false;
        use_vy_ = false;   // NORMAL：高速巡航不横移，前向+转向更稳

        to_goal_cost_gain_ = 0.9;
        obs_cost_gain_ = 0.4;
        speed_cost_gain_ = 0.1;
        path_cost_gain_ = 0.2;
        ori_cost_gain_ = 0.2;
        aw_cost_gain_ = 0.6;
        ori_shape_p_ = 1.0;   // NORMAL：朝向线性惩罚(温和，巡航不苛求对准)
    }

    void DDP::lowSpeedParameters(Robot_config &robot) {
        v_steps_ = 20;
        w_steps_ = 20;
        nr_steps_ = (int) robot.timeInterval.size();   // 步数随 timeInterval(单一来源)自适应
        nr_pairs_ = 1600;

        min_traj_length_ = 0.2;
        robot_radius_ = -0.02;

        use_goal_cost_ = true;
        use_angular_cost_ = false;
        use_path_cost_ = true;
        use_speed_cost_ = false;
        use_ori_cost_ = true;
        use_space_cost_ = true;
        use_vy_ = true;   // LOW_SPEED：低速窄道允许侧身横移挤过
        lateral_stddev = 0.1;   // 横移影响低：窄道仅轻微侧挪修正

        to_goal_cost_gain_ = 0.7;
        obs_cost_gain_ = 0.6;
        speed_cost_gain_ = 0.2;
        path_cost_gain_ = 0.4;
        ori_cost_gain_ = 0.1;
        aw_cost_gain_ = 0.4;
        space_cost_gain_ = 0.2;
        ori_shape_p_ = 1.0;   // CAUTIOUS：朝向线性惩罚(窄道以避障/贴线为主)
    }

    void DDP::recoverParameters(Robot_config &robot) {
        v_steps_ = 20;
        w_steps_ = 20;
        nr_steps_ = (int) robot.timeInterval.size();
        nr_pairs_ = 1600;

        min_traj_length_ = 0.3;   // 脱困允许几乎原地的候选(纯转向/微调挪出)
        robot_radius_ = -0.06;   // 脱困最宽松：敢贴着障碍找缝(安全已由硬斥力顶出兜底)

        use_goal_cost_ = true;
        use_angular_cost_ = false;
        use_path_cost_ = true;
        use_speed_cost_ = true;
        use_ori_cost_ = true;
        use_space_cost_ = true;
        use_vy_ = true;   // RECOVER：脱困时放开横移(蟹蟹步)
        lateral_stddev = 0.35;   // 横移影响大：卡住时敢大幅侧挪脱困

        to_goal_cost_gain_ = 0.9;
        obs_cost_gain_ = 0.6;
        speed_cost_gain_ = 0.3;
        path_cost_gain_ = 0.6;
        ori_cost_gain_ = 0.2;
        aw_cost_gain_ = 0.1;
        space_cost_gain_ = 0.1;
        ori_shape_p_ = 2.0;   // RECOVER：朝向【超线性】惩罚——大偏离(背向目标)狠罚，强逼转正
    }

    void DDP::frontBackParameters(Robot_config &robot) {
        v_steps_ = 20;
        w_steps_ = 25;
        nr_steps_ = (int) robot.timeInterval.size();
        nr_pairs_ = 800;

        min_traj_length_ = 0.05;
        robot_radius_ = 0.02;

        use_goal_cost_ = false;
        use_angular_cost_ = false;
        use_path_cost_ = true;
        use_speed_cost_ = false;
        use_ori_cost_ = false;
        use_vy_ = false;   // 前后挪车：仅沿路径前/后，不横移

        to_goal_cost_gain_ = 1.0;
        obs_cost_gain_ = 0.2;
        speed_cost_gain_ = 0.2;
        path_cost_gain_ = 0.4;
        ori_cost_gain_ = 0.5;
        aw_cost_gain_ = 0.5;
    }



    void DDPDWAPlanner::normalParameters(Robot_config &robot) {
        v_steps_ = 20;
        w_steps_ = 18;
        nr_steps_ = (int) robot.timeInterval.size();

        min_traj_length_ = 0.01;
        robot_radius_ = 0.1;

        use_goal_cost_ = true;
        use_angular_cost_ = true;
        use_path_cost_ = true;
        use_speed_cost_ = false;
        use_ori_cost_ = false;

        to_goal_cost_gain_ = 0.7;
        obs_cost_gain_ = 0.7;
        speed_cost_gain_ = 0.1;
        path_cost_gain_ = 0.7;
        ori_cost_gain_ = 0.2;
        aw_cost_gain_ = 0.2;
    }


    void DDPDWAPlanner::lowSpeedParameters(Robot_config &robot) {
        v_steps_ = 20;
        w_steps_ = 25;
        nr_steps_ = (int) robot.timeInterval.size();

        min_traj_length_ = 0.1;
        robot_radius_ = 0.1;

        use_goal_cost_ = true;
        use_angular_cost_ = true;
        use_path_cost_ = true;
        use_speed_cost_ = true;
        use_ori_cost_ = true; // close to the local goal

        to_goal_cost_gain_ = 0.8;
        obs_cost_gain_ = 0.6;
        speed_cost_gain_ = 0.2;
        path_cost_gain_ = 0.4;
        ori_cost_gain_ = 0.2;
        aw_cost_gain_ = 0.8;
    }

    void DDPDWAPlanner::recoverParameters(Robot_config &robot) {
        v_steps_ = 20;
        w_steps_ = 20;
        nr_steps_ = (int) robot.timeInterval.size();

        min_traj_length_ = 0.05;
        robot_radius_ = 0.02;

        use_goal_cost_ = true;
        use_angular_cost_ = false;
        use_path_cost_ = true;
        use_speed_cost_ = false;
        use_ori_cost_ = false;

        to_goal_cost_gain_ = 1.0;
        obs_cost_gain_ = 0.2;
        speed_cost_gain_ = 0.2;
        path_cost_gain_ = 0.4;
        ori_cost_gain_ = 0.5;
        aw_cost_gain_ = 0.5;
    }

    void DDPDWAPlanner::frontBackParameters(Robot_config &robot) {
        v_steps_ = 20;
        w_steps_ = 25;
        nr_steps_ = (int) robot.timeInterval.size();

        min_traj_length_ = 0.05;
        robot_radius_ = 0.02;

        use_goal_cost_ = false;
        use_angular_cost_ = false;
        use_path_cost_ = true;
        use_speed_cost_ = false;
        use_ori_cost_ = false;

        to_goal_cost_gain_ = 1.0;
        obs_cost_gain_ = 0.2;
        speed_cost_gain_ = 0.2;
        path_cost_gain_ = 0.4;
        ori_cost_gain_ = 0.5;
        aw_cost_gain_ = 0.5;
    }


    void DWAPlanner::normalParameters(Robot_config &robot) {
        const auto tp = robot.getTuningParams();
        v_steps_ = tp.vx_sample;
        w_steps_ = tp.vTheta_samples;
        nr_steps_ = 20;

        min_traj_length_ = 0.1;
        robot_radius_ = 0.1;

        use_goal_cost_ = true;
        use_path_cost_ = true;
        use_angular_cost_ = false;
        use_speed_cost_ = false;
        use_ori_cost_ = false;

        to_goal_cost_gain_ = goal_distance_bias;
        obs_cost_gain_ = 0.7;
        speed_cost_gain_ = 0.1;
        path_cost_gain_ = path_distance_bias;
        ori_cost_gain_ = 0.2;
        aw_cost_gain_ = 0.2;

        timeInterval.assign(nr_steps_, dt);

    }

    void DWAPlanner::lowSpeedParameters(Robot_config &robot) {
        (void)robot;
        v_steps_ = 20;
        w_steps_ = 25;
        nr_steps_ = 20;

        min_traj_length_ = 0.1;
        robot_radius_ = 0.1;

        use_goal_cost_ = true;
        use_angular_cost_ = true;
        use_path_cost_ = true;
        use_speed_cost_ = true;
        use_ori_cost_ = true; // close to the local goal

        to_goal_cost_gain_ = 0.8;
        obs_cost_gain_ = 0.6;
        speed_cost_gain_ = 0.2;
        path_cost_gain_ = 0.4;
        ori_cost_gain_ = 0.2;
        aw_cost_gain_ = 0.8;

        timeInterval.assign(nr_steps_, dt);

    }

    void DWAPlanner::recoverParameters(Robot_config &robot) {
        (void)robot;
        v_steps_ = 20;
        w_steps_ = 25;
        nr_steps_ = 20;

        min_traj_length_ = 0.05;
        robot_radius_ = 0.02;

        use_goal_cost_ = true;
        use_angular_cost_ = false;
        use_path_cost_ = true;
        use_speed_cost_ = false;
        use_ori_cost_ = false;

        to_goal_cost_gain_ = 1.0;
        obs_cost_gain_ = 0.2;
        speed_cost_gain_ = 0.2;
        path_cost_gain_ = 0.4;
        ori_cost_gain_ = 0.5;
        aw_cost_gain_ = 0.5;

        timeInterval.assign(nr_steps_, dt);

    }

    void DWAPlanner::frontBackParameters(Robot_config &robot) {
        (void)robot;
        v_steps_ = 20;
        w_steps_ = 25;
        nr_steps_ = 20;

        min_traj_length_ = 0.05;
        robot_radius_ = 0.02;

        use_goal_cost_ = false;
        use_angular_cost_ = false;
        use_path_cost_ = true;
        use_speed_cost_ = false;
        use_ori_cost_ = false;

        to_goal_cost_gain_ = 1.0;
        obs_cost_gain_ = 0.2;
        speed_cost_gain_ = 0.2;
        path_cost_gain_ = 0.4;
        ori_cost_gain_ = 0.5;
        aw_cost_gain_ = 0.5;

        timeInterval.assign(nr_steps_, dt);

    }



    void DDPMPPIPlanner::normalParameters(Robot_config &robot) {
        v_steps_ = 20;
        w_steps_ = 20;
        nr_steps_ = (int) robot.timeInterval.size();
        nr_pairs_ = 550;

        min_traj_length_ = 0.1;
        robot_radius_ = 0.1;

        use_goal_cost_ = true;
        use_angular_cost_ = true;
        use_path_cost_ = true;
        use_speed_cost_ = true;
        use_ori_cost_ = false;

        to_goal_cost_gain_ = 0.8;
        obs_cost_gain_ = 0.5;
        speed_cost_gain_ = 0.1;
        path_cost_gain_ = 0.7;
        ori_cost_gain_ = 0.2;
        aw_cost_gain_ = 0.8;
    }

    void DDPMPPIPlanner::lowSpeedParameters(Robot_config &robot) {
        v_steps_ = 20;
        w_steps_ = 20;
        nr_steps_ = (int) robot.timeInterval.size();
        nr_pairs_ = 550;

        min_traj_length_ = 0.1;
        robot_radius_ = 0.1;

        use_goal_cost_ = true;
        use_angular_cost_ = true;
        use_path_cost_ = true;
        use_speed_cost_ = true;
        use_ori_cost_ = true; // close to the local goal

        to_goal_cost_gain_ = 0.8;
        obs_cost_gain_ = 0.6;
        speed_cost_gain_ = 0.2;
        path_cost_gain_ = 0.4;
        ori_cost_gain_ = 0.2;
        aw_cost_gain_ = 0.8;
    }

    void DDPMPPIPlanner::recoverParameters(Robot_config &robot) {
        v_steps_ = 20;
        w_steps_ = 20;
        nr_steps_ = (int) robot.timeInterval.size();
        nr_pairs_ = 550;

        min_traj_length_ = 0.05;
        robot_radius_ = 0.02;

        use_goal_cost_ = true;
        use_angular_cost_ = false;
        use_path_cost_ = true;
        use_speed_cost_ = false;
        use_ori_cost_ = false;

        to_goal_cost_gain_ = 1.0;
        obs_cost_gain_ = 0.2;
        speed_cost_gain_ = 0.2;
        path_cost_gain_ = 0.4;
        ori_cost_gain_ = 0.5;
        aw_cost_gain_ = 0.5;
    }

    void DDPMPPIPlanner::frontBackParameters(Robot_config &robot) {
        v_steps_ = 20;
        w_steps_ = 20;
        nr_steps_ = (int) robot.timeInterval.size();
        nr_pairs_ = 550;

        min_traj_length_ = 0.05;
        robot_radius_ = 0.02;

        use_goal_cost_ = false;
        use_angular_cost_ = false;
        use_path_cost_ = true;
        use_speed_cost_ = false;
        use_ori_cost_ = false;

        to_goal_cost_gain_ = 1.0;
        obs_cost_gain_ = 0.2;
        speed_cost_gain_ = 0.2;
        path_cost_gain_ = 0.4;
        ori_cost_gain_ = 0.5;
        aw_cost_gain_ = 0.5;
    }


     void MPPIPlanner::normalParameters(Robot_config &robot) {
        const auto tp = robot.getTuningParams();
        v_steps_ = 20;
        w_steps_ = 20;
        nr_steps_ = tp.nr_steps_;
        nr_pairs_ = tp.nr_pairs_;
        linear_stddev = tp.linear_stddev;
        angular_stddev = tp.angular_stddev;
        lambda = tp.lambda;
        exploration_ratio = 0.8;

        min_traj_length_ = 0.01;
        robot_radius_ = 0.01;

        use_goal_cost_ = true;
        use_angular_cost_ = true;
        use_path_cost_ = true;
        use_speed_cost_ = true;
        use_ori_cost_ = false;

        to_goal_cost_gain_ = 0.8;
        obs_cost_gain_ = 0.5;
        speed_cost_gain_ = 0.1;
        path_cost_gain_ = 0.7;
        ori_cost_gain_ = 0.2;
        aw_cost_gain_ = 0.8;

        timeInterval.assign(nr_steps_, dt);
    }

    void MPPIPlanner::lowSpeedParameters(Robot_config &robot) {
        (void)robot;
        v_steps_ = 20;
        w_steps_ = 20;
        nr_steps_ = 20;
        nr_pairs_ = 600;

        min_traj_length_ = 0.01;
        robot_radius_ = 0.01;

        use_goal_cost_ = true;
        use_angular_cost_ = true;
        use_path_cost_ = true;
        use_speed_cost_ = true;
        use_ori_cost_ = true; // close to the local goal

        to_goal_cost_gain_ = 0.8;
        obs_cost_gain_ = 0.6;
        speed_cost_gain_ = 0.2;
        path_cost_gain_ = 0.4;
        ori_cost_gain_ = 0.2;
        aw_cost_gain_ = 0.8;


        timeInterval.assign(nr_steps_, dt);
    }

    void MPPIPlanner::recoverParameters(Robot_config &robot) {
        (void)robot;
        v_steps_ = 20;
        w_steps_ = 20;
        nr_steps_ = 20;
        nr_pairs_ = 600;

        min_traj_length_ = 0.05;
        robot_radius_ = 0.02;

        use_goal_cost_ = true;
        use_angular_cost_ = false;
        use_path_cost_ = true;
        use_speed_cost_ = false;
        use_ori_cost_ = false;

        to_goal_cost_gain_ = 1.0;
        obs_cost_gain_ = 0.2;
        speed_cost_gain_ = 0.2;
        path_cost_gain_ = 0.4;
        ori_cost_gain_ = 0.5;
        aw_cost_gain_ = 0.5;

        for (int i = 0; i < nr_steps_; ++i) {
            timeInterval.push_back(dt);
        }

    }

    void MPPIPlanner::frontBackParameters(Robot_config &robot) {
        (void)robot;
        v_steps_ = 20;
        w_steps_ = 20;
        nr_steps_ = 20;
        nr_pairs_ = 600;

        min_traj_length_ = 0.05;
        robot_radius_ = 0.02;

        use_goal_cost_ = false;
        use_angular_cost_ = false;
        use_path_cost_ = true;
        use_speed_cost_ = false;
        use_ori_cost_ = false;

        to_goal_cost_gain_ = 1.0;
        obs_cost_gain_ = 0.2;
        speed_cost_gain_ = 0.2;
        path_cost_gain_ = 0.4;
        ori_cost_gain_ = 0.5;
        aw_cost_gain_ = 0.5;

        for (int i = 0; i < nr_steps_; ++i) {
            timeInterval.push_back(dt);
        }

    }
}
