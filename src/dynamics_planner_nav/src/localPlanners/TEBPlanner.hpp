/*
 * TEBPlanner.hpp — Wrapper for official TEB local planner from move_base
 * Uses teb_local_planner library instead of custom implementation
 */

#ifndef Antipatrea__TEBPlanner_HPP_
#define Antipatrea__TEBPlanner_HPP_

#include "../robot/Go2.hpp"

// Official TEB planner headers (ROS2 version)
// Note: For ROS2 Humble, you need to install teb_local_planner ROS2 port
// git clone https://github.com/rst-tu-dortmund/teb_local_planner.git -b ros2
#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/visualization.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>

namespace Antipatrea {
    using PoseState = Robot_config::PoseState;

    /**
     * @brief Wrapper for official TEB (Timed Elastic Band) Local Planner
     *
     * This class wraps the official teb_local_planner from ROS move_base,
     * providing a simplified interface for use in our navigation system.
     */
    class TEBPlanner {
    public:
        TEBPlanner();
        ~TEBPlanner() = default;

        bool Solve(int nrIters, double dt, bool &canBeSolved);

        Robot_config *robot = nullptr;

    protected:
        void updateRobotState();

        void commonParameters(Robot_config &robot);
        void normalParameters(Robot_config &robot);
        void lowSpeedParameters(Robot_config &robot);

        bool handleNoMapPlanning(geometry_msgs::msg::Twist &cmd_vel);
        bool handleNormalSpeedPlanning(geometry_msgs::msg::Twist &cmd_vel, double dt);
        bool handleLowSpeedPlanning(geometry_msgs::msg::Twist &cmd_vel, double dt);
        bool handleAbnormalPlaning(geometry_msgs::msg::Twist &cmd_vel, double dt);

        void publishCommand(geometry_msgs::msg::Twist &cmd_vel, double linear, double angular);

        /**
         * @brief Call the official TEB planner
         */
        bool callTEBPlanner(geometry_msgs::msg::Twist &cmd_vel);

        /**
         * @brief Build global plan from local_goal for TEB
         */
        void buildGlobalPlan(std::vector<geometry_msgs::PoseStamped> &plan);

        /**
         * @brief Convert laser data to TEB obstacles
         */
        void buildObstacles();

        /**
         * @brief Visualize TEB trajectory
         */
        void visualizeTEBTrajectory();

        /**
         * @brief Normalize angle to [-π, π]
         */
        double normalizeAngle(double angle);

        // Official TEB planner instance
        boost::shared_ptr<teb_local_planner::TebOptimalPlanner> teb_planner_;

        // TEB configuration
        teb_local_planner::TebConfig teb_config_;

        // Obstacle container
        boost::shared_ptr<teb_local_planner::ObstContainer> obstacles_;

        // Robot footprint model
        teb_local_planner::RobotFootprintModelPtr robot_model_;

        // State tracking
        PoseState parent;
        PoseState parent_odom;
        std::vector<double> local_goal;
        std::vector<std::vector<double>> local_paths;
    };
}

#endif
