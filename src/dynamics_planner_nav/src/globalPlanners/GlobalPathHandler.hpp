// GlobalPathHandler.hpp — /plan(全局路径) 的处理逻辑，从 Go2_callbacks 抽出集中到此
//
// 注意：这不是一个 planner(不做全局搜索)，只负责把已经算好的 /plan 入库到 Robot_config
// 的双坐标系容器，并由 LocalGoalGenerator 生成 local goal。两个入口：
//   processValidGlobalPath —— 非空路径：双坐标系入库 + 生成 local goal + 目标可视化。
//   processEmptyGlobalPath —— 空路径：目前仅切 BRAKE(占位，后续可扩展空路径策略)。
// globalPathCallback 只按 poses 是否为空做分流，逻辑实体全在这里。
//
// 需要访问 Robot_config 的私有成员(global_paths_*、path_goal_mutex_ 等)，故在
// Robot_config 中声明为 friend(与 Go2Callbacks 同一模式)。

#ifndef DYNAMICS_PLANNER_NAV_GLOBAL_PATH_HANDLER_HPP
#define DYNAMICS_PLANNER_NAV_GLOBAL_PATH_HANDLER_HPP

#include <nav_msgs/msg/path.hpp>

// Forward declaration
class Robot_config;

class GlobalPathHandler {
public:
    // 非空 /plan：入库 + 生成 local goal + 发布目标可视化。
    static void processValidGlobalPath(Robot_config* robot,
                                       const nav_msgs::msg::Path::SharedPtr& msg);

    // 空 /plan：占位策略(当前切 BRAKE)。目前用不上，先备好接口。
    static void processEmptyGlobalPath(Robot_config* robot);
};

#endif  // DYNAMICS_PLANNER_NAV_GLOBAL_PATH_HANDLER_HPP
