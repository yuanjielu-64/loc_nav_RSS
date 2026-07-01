// LocalPlannerBase.cpp — 局部规划器公共基类【实现】
//
// 这里的实现是从各 planner(以 DDP 为准)逐字搬来的通用纯函数，保证行为完全一致。

#include "localPlanners/LocalPlannerBase.hpp"

namespace Antipatrea {

    // 差速版：linear.x + angular.z，linear.y 强制 0。供不需要侧移的下发场景使用。
    void LocalPlannerBase::publishCommand(Robot_config &robot, geometry_msgs::msg::Twist &cmd_vel,
                                          double linear, double angular) {
        cmd_vel.linear.x = linear;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = angular;
        robot.Control()->publish(cmd_vel);
    }

    // 全向版：vx 前向 + vy 侧向 + angular 偏航。节流打印 [cmd][状态] 便于联调。
    void LocalPlannerBase::publishCommand(Robot_config &robot, geometry_msgs::msg::Twist &cmd_vel,
                                          double vx, double vy, double angular) {
        cmd_vel.linear.x = vx;
        cmd_vel.linear.y = vy;
        cmd_vel.angular.z = angular;
        robot.Control()->publish(cmd_vel);

        // 状态枚举 -> 短名，便于日志里看当前模式。
        auto stateName = [](Robot_config::RobotState s) -> const char* {
            switch (s) {
                case Robot_config::INIT:     return "INIT";
                case Robot_config::NORMAL:   return "NORMAL";
                case Robot_config::CAUTIOUS: return "LOW";
                case Robot_config::BLIND:    return "NO_MAP";
                case Robot_config::BRAKE:    return "BRAKE";
                case Robot_config::RECOVER:  return "RECOVER";
                case Robot_config::ROTATE:   return "ROTATE";
                case Robot_config::BACK:     return "BACK";
                default:                     return "OTHER";
            }
        };

        const auto cur = robot.getPoseState();
        // [cmd] 每次下发都打会刷屏、淹没 RECOVER 阶段日志 → 降为 DEBUG(默认隐藏)，
        // 需要看每帧实际下发速度时把日志级别设为 DEBUG 即可调出。
        RCLCPP_DEBUG_THROTTLE(
            robot.get_logger(), *robot.get_clock(), 500,
            "[cmd][%s] vx=%.3f vy=%.3f w=%.3f | cur_vel=%.3f cur_vy=%.3f",
            stateName(robot.getRobotState()),
            vx, vy, angular, cur.vx_, cur.vy_);
    }

    // 通用动态窗口：基于当前状态 + 加减速度常量 + 速度限制(robot->getVelocityLimits)。
    // 沿用 DDP 那版(含 vy 段)；MPPI 等差速 planner 因 max_lateral=0、计算后仍是 0，行为不变。
    LocalPlannerBase::Window LocalPlannerBase::calc_dynamic_window(
        Robot_config &robot, const PoseState &state, double dt) const {
        Window window;
        auto velocity = robot.getVelocityLimits();

        window.min_velocity_ = std::max((state.vx_ + minAccelerSpeed * dt), velocity.min_linear);
        window.max_velocity_ = std::min((state.vx_ + maxAccelerSpeed * dt), velocity.max_linear);

        window.min_angular_velocity_ = std::max(
            (state.angular_velocity_ + minAngularAccelerSpeed * dt), velocity.min_angular);
        window.max_angular_velocity_ = std::min(
            (state.angular_velocity_ + maxAngularAccelerSpeed * dt), velocity.max_angular);

        // 全向侧移窗口：vy 暂用与前向相同的加减速度上下限(差速车 velocity.min/max_lateral=0)。
        window.min_lateral_velocity_ = std::max(
            (state.vy_ + minAccelerSpeed * dt), velocity.min_lateral);
        window.max_lateral_velocity_ = std::min(
            (state.vy_ + maxAccelerSpeed * dt), velocity.max_lateral);

        return window;
    }

    double LocalPlannerBase::normalizeAngle(double angle) {
        angle = fmod(angle + M_PI, 2 * M_PI);
        if (angle <= 0)
            angle += 2 * M_PI;
        return angle - M_PI;
    }

    double LocalPlannerBase::updateVelocity(double current, double target,
                                            double maxAccel, double minAccel, double t) {
        if (current < target) {
            return std::min(current + maxAccel * t, target);
        } else {
            return std::max(current + minAccel * t, target);
        }
    }

    double LocalPlannerBase::calculateTheta(const PoseState &state, const double *y) const {
        double deltaX = y[0] - state.x_;
        double deltaY = y[1] - state.y_;
        double theta = atan2(deltaY, deltaX);

        double normalizedTheta = normalizeAngle(state.theta_);

        return normalizeAngle(theta - normalizedTheta);
    }

    double LocalPlannerBase::calc_angular_velocity(const std::vector<PoseState> &traj) const {
        if (use_angular_cost_) {
            double angular_velocity = std::abs(traj.front().angular_velocity_);
            double angular_velocity_cost = angular_velocity * angular_velocity;
            return angular_velocity_cost;
        }
        return 0.0;
    }

    // 通用初始化：5 个 planner 原 commonParameters 字节级一致(都调 setCommonPlannerParameters
    // helper + 设 local_goal_odom)。这里把那个 helper 的内容内联进来，逻辑完全等价。
    void LocalPlannerBase::commonParameters(Robot_config &robot) {
        const auto tp = robot.getTuningParams();
        dt = tp.dt;
        minAccelerSpeed = -20.0;
        maxAccelerSpeed = 20.0;
        minAngularAccelerSpeed = -25;
        maxAngularAccelerSpeed = 25;
        // DDP 系 planner 的 traj 是 odom 帧，故取 odom 版 local goal(getLocalGoalOdomCfg)。
        // base_link 版(getLocalGoalCfg)给 TEB 用。
        local_goal_odom = robot.getLocalGoalOdomCfg();
    }

    // ---- RobotBox ----
    LocalPlannerBase::RobotBox::RobotBox() : x_min(0.0), x_max(0.0), y_min(0.0), y_max(0.0) {}
    LocalPlannerBase::RobotBox::RobotBox(double x_min_, double x_max_, double y_min_, double y_max_)
        : x_min(x_min_), x_max(x_max_), y_min(y_min_), y_max(y_max_) {}

    // ---- Window ----
    LocalPlannerBase::Window::Window()
        : min_velocity_(0.0), max_velocity_(0.0),
          min_angular_velocity_(0.0), max_angular_velocity_(0.0),
          min_lateral_velocity_(0.0), max_lateral_velocity_(0.0) {}

    void LocalPlannerBase::Window::show() const {
        std::cout << "[INFO] Window:" << std::endl;
        std::cout << "[INFO] \tVelocity:" << std::endl;
        std::cout << "[INFO] \t\tmax: " << max_velocity_ << std::endl;
        std::cout << "[INFO] \t\tmin: " << min_velocity_ << std::endl;
        std::cout << "[INFO] \tYawrate:" << std::endl;
        std::cout << "[INFO] \t\tmax: " << max_angular_velocity_ << std::endl;
        std::cout << "[INFO] \t\tmin: " << min_angular_velocity_ << std::endl;
    }

    // ---- Cost ----
    LocalPlannerBase::Cost::Cost()
        : obs_cost_(0.0), to_goal_cost_(0.0), speed_cost_(0.0), path_cost_(0.0),
          ori_cost_(0.0), aw_cost_(0.0), space_cost_(0.0), total_cost_(0.0) {}

    // 7 参版本：space_cost_ 默认 0(其它 4 个 planner 原行为)。
    LocalPlannerBase::Cost::Cost(
        const double obs_cost, const double to_goal_cost, const double speed_cost,
        const double path_cost, const double ori_cost, const double aw_cost,
        const double total_cost)
        : obs_cost_(obs_cost), to_goal_cost_(to_goal_cost), speed_cost_(speed_cost),
          path_cost_(path_cost), ori_cost_(ori_cost), aw_cost_(aw_cost),
          space_cost_(0.0), total_cost_(total_cost) {}

    // 8 参版本：含 space_cost_(DDP 原行为)。
    LocalPlannerBase::Cost::Cost(
        const double obs_cost, const double to_goal_cost, const double speed_cost,
        const double path_cost, const double ori_cost, const double aw_cost,
        const double space_cost, const double total_cost)
        : obs_cost_(obs_cost), to_goal_cost_(to_goal_cost), speed_cost_(speed_cost),
          path_cost_(path_cost), ori_cost_(ori_cost), aw_cost_(aw_cost),
          space_cost_(space_cost), total_cost_(total_cost) {}

    void LocalPlannerBase::Cost::show() const {
        std::cout << "[INFO] Cost: " << total_cost_ << std::endl;
        std::cout << "[INFO] \tObs cost: " << obs_cost_ << std::endl;
        std::cout << "[INFO] \tGoal cost: " << to_goal_cost_ << std::endl;
        std::cout << "[INFO] \tSpeed cost: " << speed_cost_ << std::endl;
        std::cout << "[INFO] \tPath cost: " << path_cost_ << std::endl;
        std::cout << "[INFO] \tOri cost: " << ori_cost_ << std::endl;
        std::cout << "[INFO] \tSpace cost: " << space_cost_ << std::endl;
    }

    // 行为兼容：对从不设置 space_cost_ 的 planner，它默认 0，加 0 不变；
    // 对 DDP 则加上其计算出的 space_cost_，与原 DDP::Cost::calc_total_cost 等价。
    void LocalPlannerBase::Cost::calc_total_cost() {
        total_cost_ = obs_cost_ + to_goal_cost_ + speed_cost_ + path_cost_ + ori_cost_ + space_cost_;
    }

}  // namespace Antipatrea

