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
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
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
            default_value='true',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'planner',
            default_value='DWA',
            description='Planner to use: DWA, MPPI, DDP, TEB, DWA_DDP, MPPI_DDP, TEB_DDP'
        ),
        dynamics_planner_node,
    ])
