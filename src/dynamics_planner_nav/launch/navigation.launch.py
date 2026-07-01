#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
def generate_launch_description():
    pkg_share = get_package_share_directory('dynamics_planner_nav')
    # Declare arguments
    # 真机(Unitree Go2)运行：订阅 /utlidar/robot_odom 真实里程计、无 Gazebo /clock。
    # 必须用墙钟，否则节点等 /clock 而 TF(base_link 由狗端 robot_state_publisher 用墙钟/
    # 单调时钟打戳)时间源不一致 → TF_OLD_DATA "data from the past"。仿真时显式传 true 覆盖。
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    planner = LaunchConfiguration('planner', default='DWA')
    # Dynamics planner navigation node
    dynamics_planner_node = Node(
        package='dynamics_planner_nav',
        executable='dynamics_planner_nav_node',
        name='dynamics_planner_nav',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'planner': planner,
        }],
        remappings=[
            # 里程计已在代码中直接订阅 /utlidar/robot_odom，无需在此 remap
            # 激光话题（如有不同，改右侧目标即可）
            ('/front/scan', '/front/scan'),
        ]
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true. 真机务必 false(无 /clock)；仅 Gazebo 仿真传 true。'
        ),
        DeclareLaunchArgument(
            'planner',
            default_value='DWA',
            description='Planner to use: DWA, MPPI, DDP, TEB, DWA_DDP, MPPI_DDP, TEB_DDP'
        ),
        dynamics_planner_node,
    ])
