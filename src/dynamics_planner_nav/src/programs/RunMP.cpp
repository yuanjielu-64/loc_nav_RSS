// RunMP.cpp - Main entry point for dynamics planner navigation (ROS2 Humble)

#include "../robot/Go2.hpp"
#include "../localPlanners/DDP.hpp"
#include "../localPlanners/DWAPlanner.hpp"
#include "../localPlanners/DWA_DDPPlanner.hpp"
#include "../localPlanners/MPPIPlanner.hpp"
#include "../localPlanners/MPPI_DDPPlanner.hpp"
#ifdef USE_TEB_PLANNER
#include "../localPlanners/TEBPlanner.hpp"
#include "../localPlanners/TEB_DDPPlanner.hpp"
#endif
#include "../utils/Timer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <string>
#include <algorithm>
#include <cstdlib>

namespace {
    Robot_config::Algorithm parse_algorithm(std::string s) {
        std::transform(s.begin(), s.end(), s.begin(),
            [](unsigned char c){ return static_cast<char>(::toupper(c)); });
        std::replace(s.begin(), s.end(), '-', '_');

        static const std::unordered_map<std::string, Robot_config::Algorithm> kLut = {
            {"DDP",        Robot_config::DDP},
            {"DWA",        Robot_config::DWA},
            {"DWA_DDP",    Robot_config::DWA_DDP},
            {"MPPI",       Robot_config::MPPI},
            {"MPPI_DDP",   Robot_config::MPPI_DDP},
            {"TEB",        Robot_config::TEB},
            {"TEB_DDP",    Robot_config::TEB_DDP},
        };
        const auto it = kLut.find(s);
        return it == kLut.end() ? Robot_config::DDP : it->second;
    }
}

int main(int argc, char **argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Resolve planner selection + max speed from CLI
    std::string planner_arg;
    double max_speed = -1.0;   // <0 表示未指定
    for (int i = 1; i + 1 < argc; ++i) {
        const std::string flag = argv[i];
        if (flag == "--planner" || flag == "-p") {
            planner_arg = argv[i + 1];
        } else if (flag == "--max-speed" || flag == "-m") {
            max_speed = std::atof(argv[i + 1]);
        }
    }

    // Create robot node. -m(桥最大线速度)在构造时定好；未指定则用默认 2.0。
    const double bridge_max_vel = (max_speed > 0.0) ? max_speed : 2.0;
    auto robot = std::make_shared<Robot_config>(bridge_max_vel);
    RCLCPP_INFO(robot->get_logger(), "Bridge max velocity = %.2f m/s (from -m)", bridge_max_vel);

    // Fallback to ROS2 parameter if not set via CLI
    if (planner_arg.empty()) {
        robot->declare_parameter("planner", "DWA");
        planner_arg = robot->get_parameter("planner").as_string();
    }

    const auto algo = parse_algorithm(planner_arg);
    robot->setAlgorithm(algo);


    double n = 20;  // 20 Hz
    robot->setDt(1.0/n);

    rclcpp::Rate rate(n);

    // Instantiate selected planner and bind solve step
    std::function<void()> solve_step;
    switch (algo) {
        case Robot_config::DWA: {
            static Antipatrea::DWAPlanner planner;
            planner.robot = robot.get();
            RCLCPP_INFO(robot->get_logger(), "dynamics_planner_nav using DWA planner");
            solve_step = [&](){ (void)planner.Solve(1, robot->getDt(), robot->canBeSolved); };
            break;
        }
        case Robot_config::DWA_DDP: {
            static Antipatrea::DDPDWAPlanner planner;
            planner.robot = robot.get();
            RCLCPP_INFO(robot->get_logger(), "dynamics_planner_nav using DDPDWA planner");
            solve_step = [&](){ (void)planner.Solve(1, robot->getDt(), robot->canBeSolved); };
            break;
        }
        case Robot_config::MPPI: {
            static Antipatrea::MPPIPlanner planner;
            planner.robot = robot.get();
            RCLCPP_INFO(robot->get_logger(), "dynamics_planner_nav using MPPI planner");
            solve_step = [&](){ (void)planner.Solve(1, robot->getDt(), robot->canBeSolved); };
            break;
        }
        case Robot_config::MPPI_DDP: {
            static Antipatrea::DDPMPPIPlanner planner;
            planner.robot = robot.get();
            RCLCPP_INFO(robot->get_logger(), "dynamics_planner_nav using DDPMPPI planner");
            solve_step = [&](){ (void)planner.Solve(1, robot->getDt(), robot->canBeSolved); };
            break;
        }
#ifdef USE_TEB_PLANNER
        case Robot_config::TEB: {
            static Antipatrea::TEBPlanner planner;
            planner.robot = robot.get();
            RCLCPP_INFO(robot->get_logger(), "dynamics_planner_nav using TEB planner");
            solve_step = [&](){ (void)planner.Solve(1, robot->getDt(), robot->canBeSolved); };
            break;
        }
        case Robot_config::TEB_DDP: {
            static Antipatrea::TEB_DDPPlanner planner;
            planner.robot = robot.get();
            RCLCPP_INFO(robot->get_logger(), "Switched to TEB_DDP planner");
            solve_step = [&](){ (void)planner.Solve(1, robot->getDt(), robot->canBeSolved); };
            break;
        }
#endif
        case Robot_config::DDP:
        default: {
            static Antipatrea::DDP planner;
            planner.robot = robot.get();
            RCLCPP_INFO(robot->get_logger(), "dynamics_planner_nav using DDP planner");
            solve_step = [&](){ (void)planner.Solve(1, robot->getDt(), robot->canBeSolved); };
            break;
        }
    }

    while (rclcpp::ok()) {
        // 记录本次迭代的工作耗时(spin_some + setup + solve，不含 rate.sleep 的等待)。
        Antipatrea::Timer::Clock loop_clk;
        Antipatrea::Timer::Start(loop_clk);

        rclcpp::spin_some(robot);

        if (!robot->setup()) {
            if (robot->getRobotState() == Robot_config::BRAKE)
                rate.sleep();
            continue;
        }

        // Call selected planner solve function
        solve_step();

        // 本次迭代工作耗时(ms)。属性能诊断、与脱困分析无关 → 降为 DEBUG(默认隐藏)，避免刷屏。
        const double loop_ms = Antipatrea::Timer::Elapsed(loop_clk) * 1000.0;
        RCLCPP_DEBUG_THROTTLE(robot->get_logger(), *robot->get_clock(), 1000,
                             "[loop] 本次迭代耗时=%.2f ms (预算 %.1f ms @ %.0f Hz)",
                             loop_ms, 1000.0 / n, n);

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
