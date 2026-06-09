// SwitchMP.cpp - ROS2 Humble version
// Dynamic planner switching based on topic input

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
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <unordered_map>
#include <string>
#include <algorithm>
#include <memory>
#include <functional>

namespace {
    Robot_config::Algorithm parse_algorithm(std::string s, bool &valid) {
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return static_cast<char>(::toupper(c)); });
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
        valid = (it != kLut.end());
        return valid ? it->second : Robot_config::DDP;
    }
}

class SwitchMPNode : public rclcpp::Node {
public:
    SwitchMPNode(const std::string& initial_planner = "")
        : Node("switch_mp_node"),
          g_current_algo_(Robot_config::DDP),
          g_planner_initialized_(false),
          g_planner_changed_(false)
    {
        // Declare parameter for planner
        this->declare_parameter<std::string>("planner", "");

        // Get planner from parameter if not provided via constructor
        std::string planner_arg = initial_planner;
        if (planner_arg.empty()) {
            this->get_parameter("planner", planner_arg);
        }

        // Create robot instance
        robot_ = std::make_shared<Robot_config>();

        // Check if planner was specified
        if (!planner_arg.empty()) {
            bool valid = false;
            g_current_algo_ = parse_algorithm(planner_arg, valid);
            if (valid) {
                g_planner_initialized_ = true;
                robot_->setAlgorithm(g_current_algo_);
                RCLCPP_INFO(this->get_logger(), "Initial planner set to: %s", planner_arg.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid planner specified: %s. Waiting for valid planner via /planner_switch topic...", planner_arg.c_str());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "No planner specified. Waiting for planner via /planner_switch topic...");
        }

        double n = 20.0;  // 20 Hz
        robot_->setDt(1.0/n);

        // Subscribe to planner switch topic
        planner_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/planner_switch", 10,
            std::bind(&SwitchMPNode::plannerSwitchCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to /planner_switch topic for dynamic planner switching");

        // Create timer for main loop (20 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0/n)),
            std::bind(&SwitchMPNode::timerCallback, this));

        // Initialize planner only if one was specified
        if (g_planner_initialized_) {
            setupPlanner();
        }
    }

private:
    void plannerSwitchCallback(const std_msgs::msg::String::SharedPtr msg) {
        bool valid = false;
        auto new_algo = parse_algorithm(msg->data, valid);
        if (valid && (new_algo != g_current_algo_ || !g_planner_initialized_)) {
            g_current_algo_ = new_algo;
            g_planner_initialized_ = true;
            g_planner_changed_ = true;
            RCLCPP_INFO(this->get_logger(), "Received planner switch request: %s -> Algorithm ID: %d",
                        msg->data.c_str(), static_cast<int>(new_algo));
        }
    }

    void setupPlanner() {
        robot_->setAlgorithm(g_current_algo_);

        switch (g_current_algo_) {
            case Robot_config::DWA: {
                if (!dwa_planner_) dwa_planner_ = std::make_unique<Antipatrea::DWAPlanner>();
                dwa_planner_->robot = robot_.get();
                RCLCPP_INFO(this->get_logger(), "Switched to DWA planner");
                solve_step_ = [this](){ (void)dwa_planner_->Solve(1, robot_->getDt(), robot_->canBeSolved); };
                break;
            }
            case Robot_config::DWA_DDP: {
                if (!dwa_ddp_planner_) dwa_ddp_planner_ = std::make_unique<Antipatrea::DDPDWAPlanner>();
                dwa_ddp_planner_->robot = robot_.get();
                RCLCPP_INFO(this->get_logger(), "Switched to DWA_DDP planner");
                solve_step_ = [this](){ (void)dwa_ddp_planner_->Solve(1, robot_->getDt(), robot_->canBeSolved); };
                break;
            }
            case Robot_config::MPPI: {
                if (!mppi_planner_) mppi_planner_ = std::make_unique<Antipatrea::MPPIPlanner>();
                mppi_planner_->robot = robot_.get();
                RCLCPP_INFO(this->get_logger(), "Switched to MPPI planner");
                solve_step_ = [this](){ (void)mppi_planner_->Solve(1, robot_->getDt(), robot_->canBeSolved); };
                break;
            }
            case Robot_config::MPPI_DDP: {
                if (!mppi_ddp_planner_) mppi_ddp_planner_ = std::make_unique<Antipatrea::DDPMPPIPlanner>();
                mppi_ddp_planner_->robot = robot_.get();
                RCLCPP_INFO(this->get_logger(), "Switched to MPPI_DDP planner");
                solve_step_ = [this](){ (void)mppi_ddp_planner_->Solve(1, robot_->getDt(), robot_->canBeSolved); };
                break;
            }
#ifdef USE_TEB_PLANNER
            case Robot_config::TEB: {
                if (!teb_planner_) teb_planner_ = std::make_unique<Antipatrea::TEBPlanner>();
                teb_planner_->robot = robot_.get();
                RCLCPP_INFO(this->get_logger(), "Switched to TEB planner");
                solve_step_ = [this](){ (void)teb_planner_->Solve(1, robot_->getDt(), robot_->canBeSolved); };
                break;
            }
            case Robot_config::TEB_DDP: {
                if (!teb_ddp_planner_) teb_ddp_planner_ = std::make_unique<Antipatrea::TEB_DDPPlanner>();
                teb_ddp_planner_->robot = robot_.get();
                RCLCPP_INFO(this->get_logger(), "Switched to TEB_DDP planner");
                solve_step_ = [this](){ (void)teb_ddp_planner_->Solve(1, robot_->getDt(), robot_->canBeSolved); };
                break;
            }
#endif
            case Robot_config::DDP:
            default: {
                if (!ddp_planner_) ddp_planner_ = std::make_unique<Antipatrea::DDP>();
                ddp_planner_->robot = robot_.get();
                RCLCPP_INFO(this->get_logger(), "Switched to DDP planner");
                solve_step_ = [this](){ (void)ddp_planner_->Solve(1, robot_->getDt(), robot_->canBeSolved); };
                break;
            }
        }
        g_planner_changed_ = false;
    }

    void timerCallback() {
        // Wait until planner is initialized
        if (!g_planner_initialized_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "No planner initialized. Waiting for planner specification via /planner_switch topic...");
            return;
        }

        // Check if planner needs to be switched
        if (g_planner_changed_) {
            setupPlanner();
        }

        if (!robot_->setup()) {
            // Robot not ready yet
            return;
        }

        // Call selected planner solve function
        if (solve_step_) {
            solve_step_();
        }
    }

    // ROS2 members
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Robot
    std::shared_ptr<Robot_config> robot_;

    // Planner state
    Robot_config::Algorithm g_current_algo_;
    bool g_planner_initialized_;
    bool g_planner_changed_;
    std::function<void()> solve_step_;

    // Planners (lazy initialized)
    std::unique_ptr<Antipatrea::DDP> ddp_planner_;
    std::unique_ptr<Antipatrea::DWAPlanner> dwa_planner_;
    std::unique_ptr<Antipatrea::DDPDWAPlanner> dwa_ddp_planner_;
    std::unique_ptr<Antipatrea::MPPIPlanner> mppi_planner_;
    std::unique_ptr<Antipatrea::DDPMPPIPlanner> mppi_ddp_planner_;
#ifdef USE_TEB_PLANNER
    std::unique_ptr<Antipatrea::TEBPlanner> teb_planner_;
    std::unique_ptr<Antipatrea::TEB_DDPPlanner> teb_ddp_planner_;
#endif
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Parse command line for --planner or -p flag
    std::string planner_arg;
    for (int i = 1; i + 1 < argc; ++i) {
        const std::string flag = argv[i];
        if (flag == "--planner" || flag == "-p") {
            planner_arg = argv[i + 1];
            break;
        }
    }

    auto node = std::make_shared<SwitchMPNode>(planner_arg);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
