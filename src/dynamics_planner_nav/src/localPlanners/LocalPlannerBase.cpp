// LocalPlannerBase.cpp — 局部规划器公共基类【实现】
//
// 这里的实现是从各 planner(以 DDP 为准)逐字搬来的通用纯函数，保证行为完全一致。

#include "localPlanners/LocalPlannerBase.hpp"

namespace Antipatrea {

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
    // helper + 设 local_goal)。这里把那个 helper 的内容内联进来，逻辑完全等价。
    void LocalPlannerBase::commonParameters(Robot_config &robot) {
        const auto tp = robot.getTuningParams();
        dt = tp.dt;
        minAccelerSpeed = -20.0;
        maxAccelerSpeed = 20.0;
        minAngularAccelerSpeed = -25;
        maxAngularAccelerSpeed = 25;
        local_goal = robot.getLocalGoalCfg();
    }

    // ---- RobotBox ----
    LocalPlannerBase::RobotBox::RobotBox() : x_min(0.0), x_max(0.0), y_min(0.0), y_max(0.0) {}
    LocalPlannerBase::RobotBox::RobotBox(double x_min_, double x_max_, double y_min_, double y_max_)
        : x_min(x_min_), x_max(x_max_), y_min(y_min_), y_max(y_max_) {}

    // ---- Window ----
    LocalPlannerBase::Window::Window()
        : min_velocity_(0.0), max_velocity_(0.0),
          min_angular_velocity_(0.0), max_angular_velocity_(0.0) {}

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

