// LocalPlannerBase.hpp — 局部规划器(local planners)的公共基类【声明】
//
// 设计目标：把各 planner(DDP/DWA/MPPI/...) 里【完全相同、与具体算法无关】的
// 通用工具收进这个基类，让它们继承共用，避免一份代码复制 N 遍。
//
// 放进来的原则(很重要)：
//   ✅ 只放"各 planner 完全一样"的纯计算/通用工具(如角度归一化、速度更新)。
//   ❌ 不放"每个 planner 各不相同"的策略(如 calc_obs_cost / calc_to_goal_cost)——
//      那些留在各自 planner 里，需要时各自 override。
//
// 迁移策略：一次只让一个 planner 继承、编译+实机验证通过后再推进下一个。
// 目前是试点阶段：仅 DDP 继承本基类。

#ifndef ANTIPATREA_LOCAL_PLANNER_BASE_HPP_
#define ANTIPATREA_LOCAL_PLANNER_BASE_HPP_

#include "../robot/Go2.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

namespace Antipatrea {

    class LocalPlannerBase {
    public:
        using PoseState = Robot_config::PoseState;

        LocalPlannerBase() = default;
        virtual ~LocalPlannerBase() = default;

        // 公共的二维轴对齐盒(各 planner 原 RobotBox 字节级一致，提到基类)。
        class RobotBox {
        public:
            RobotBox();
            RobotBox(double x_min_, double x_max_, double y_min_, double y_max_);

            double x_min, x_max;
            double y_min, y_max;
        };

        // 公共的速度/角速度动态窗口(各 planner 原 Window 字段一致，提到基类)。
        // show() 仅 DDP 历史上有定义，提到基类供需要时调用，行为不变。
        class Window {
        public:
            Window();

            void show() const;

            double min_velocity_;
            double max_velocity_;
            double min_angular_velocity_;
            double max_angular_velocity_;
        };

        // 公共的轨迹代价结构(各 planner 原 Cost 字段几乎一致，DDP 多 space_cost_)。
        // 兼容两种构造方式：
        //   - 7 参：无 space_cost(其它 4 个 planner 原写法)；space_cost_ 默认 0
        //   - 8 参：含 space_cost(DDP 原写法)
        // calc_total_cost 含 space_cost_，对从不设置它的 planner space_cost_=0，行为不变。
        class Cost {
        public:
            Cost();

            // 7 参：保持原 DWA/MPPI/DWA_DDP/MPPI_DDP 调用接口不变。
            Cost(double obs_cost, double to_goal_cost, double speed_cost, double path_cost,
                 double ori_cost, double aw_cost, double total_cost);

            // 8 参：保持原 DDP 调用接口不变(含 space_cost)。
            Cost(double obs_cost, double to_goal_cost, double speed_cost, double path_cost,
                 double ori_cost, double aw_cost, double space_cost, double total_cost);

            void show() const;
            void calc_total_cost();

            double obs_cost_;
            double to_goal_cost_;
            double speed_cost_;
            double path_cost_;
            double ori_cost_;
            double aw_cost_;
            double space_cost_;   // 仅 DDP 使用；其它 planner 默认 0、对总代价无影响
            double total_cost_;
        };

    protected:
        // 角度归一化到 [-pi, pi)。纯函数，与具体 planner 无关。
        static double normalizeAngle(double angle);

        // 带加/减速度限制的速度更新：朝 target 逼近，受 maxAccel/minAccel 限幅。
        // 纯函数，与具体 planner 无关。
        static double updateVelocity(double current, double target,
                                     double maxAccel, double minAccel, double t);

        // 计算从 state 朝目标点 y(x,y) 的相对朝向角(已归一化)。纯几何，无实例状态。
        double calculateTheta(const PoseState &state, const double *y) const;

        // 角速度代价：use_angular_cost_ 开启时返回 |w|^2，否则 0。各 planner 完全一致。
        double calc_angular_velocity(const std::vector<PoseState> &traj) const;

        // 通用初始化：从 Robot_config 取 dt + 默认加减速度常量 + local_goal。
        // 5 个 planner 原 commonParameters 字节级一致，统一到基类。
        // 各 planner 主循环每次 solve 前调用一次。
        void commonParameters(Robot_config &robot);

        // 是否启用角速度代价(由各 planner 的 *Parameters 设置)。原本每个 planner 各有一份，
        // 现统一到基类，calc_angular_velocity 直接用。
        bool use_angular_cost_ = false;

        // ---- 通用运动/求解状态(原各 planner 重复，提到基类) ----
        std::vector<double> local_goal;   // 当前 local goal(odom 帧 x,y)，commonParameters 设置
        double dt{};                       // 控制 dt(从 Robot_config 取)
        double minAccelerSpeed{};          // 加减速度常量(commonParameters 设置)
        double maxAccelerSpeed{};
        double minAngularAccelerSpeed{};
        double maxAngularAccelerSpeed{};
    };

}  // namespace Antipatrea

#endif  // ANTIPATREA_LOCAL_PLANNER_BASE_HPP_

