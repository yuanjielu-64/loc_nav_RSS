"""
navigation_self_ctrl.launch.py — 只提供“全局路径”，不执行导航。

设计：自定义局部规划器(dynamics_planner_nav)独占 /cmd_vel，Nav2 这边只负责
持续产出一条全局路径 /plan，不做 FollowPath / 进度检查 / abort / 恢复。

  * 只启动 planner_server(出 /plan + global_costmap) 和 controller_server
    (仅为提供 /local_costmap/costmap，其速度输出 remap 到死话题 cmd_vel_nav)。
  * 不启动 bt_navigator / behavior_server / smoother / waypoint_follower
    —— 这些“执行导航”的组件会在机器人不动时判失败并掐掉 /plan。
  * global_path_provider 周期性调用 compute_path_to_pose，让 /plan 常驻输出。
  * cmd_vel_to_sport 把自定义 planner 的 /cmd_vel 转给 Go2 sport API。

用法：
  1. 先起定位栈(提供 /scan、odom->base_link TF、/odometry/filtered)
  2. ros2 launch nav_loc_navigation navigation_self_ctrl.launch.py
  3. 起你的 dynamics_planner_nav 节点
  4. 发一个目标到 /goal_pose(RViz 的 2D Goal Pose 即可) -> /plan 持续输出
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nav_loc_navigation')
    default_nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    params_file = LaunchConfiguration('params_file')
    # 是否启动 cmd_vel_to_sport 桥（把 /cmd_vel 转给真狗）。
    # 默认 false：只产出 /plan + costmap，绝不驱动狗，便于安全调试。
    # 真要让狗动时显式加 use_cmd_bridge:=true。
    use_cmd_bridge = LaunchConfiguration('use_cmd_bridge')

    # 只做“全局路径供给”：planner_server 出 /plan + global_costmap；
    # controller_server 仅为提供 /local_costmap/costmap（速度已 remap 到死话题）。
    # 不启动 bt_navigator/behavior_server 等“执行导航”的组件——它们会在机器人
    # 不动时判失败并掐掉 /plan。global_path_provider 周期性重算，让 /plan 常驻。
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
    ]

    common_params = [params_file, {'use_sim_time': use_sim_time}]
    log_args = ['--ros-args', '--log-level', log_level]

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_nav2_params,
            description='Nav2 parameters YAML owned by nav_loc_navigation.'),
        DeclareLaunchArgument(
            'use_cmd_bridge', default_value='false',
            description='true 时才启动 cmd_vel_to_sport 桥(会驱动真狗)；'
                        '默认 false 只产出 /plan+costmap，不驱动狗。'),
    ]

    nodes = [
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # Hosts local_costmap (/local_costmap/costmap). Its velocity output is
        # remapped to a dead topic so it never fights the custom planner.
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=common_params,
            arguments=log_args,
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        # Hosts global_costmap + produces /plan.
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=common_params,
            arguments=log_args,
        ),
        # 周期性调用 planner 的 compute_path_to_pose，让 /plan 常驻输出。
        Node(
            package='nav_loc_navigation',
            executable='global_path_provider',
            name='global_path_provider',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'rate': 5.0,
                'planner_id': 'GridBased',
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=log_args,
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
            }],
        ),

        # Relay the CUSTOM planner's /cmd_vel to the Go2 sport API.
        # 仅当 use_cmd_bridge:=true 时启动；默认不启动 -> 狗不会动。
        Node(
            package='nav_loc_navigation',
            executable='cmd_vel_to_sport',
            name='cmd_vel_to_sport',
            output='screen',
            condition=IfCondition(use_cmd_bridge),
        ),
    ]

    return LaunchDescription(args + nodes)

