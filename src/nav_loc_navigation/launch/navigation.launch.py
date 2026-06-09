"""
navigation.launch.py — start the map-less Nav2 stack + cmd_vel->sport bridge.

Brings up the official nav2_bringup navigation_launch.py (controller, planner,
behaviors, bt_navigator, smoother, waypoint_follower, velocity_smoother and the
lifecycle manager) configured with our map-less params (config/nav2_params.yaml,
both costmaps rolling in the odom frame — no map server / AMCL), then starts
cmd_vel_to_sport so Nav2's /cmd_vel drives the Go2 via the sport API.

Needs the localization stack running first (start_stack.sh) for /scan and the
odom->base_link TF.

Run with:
    ros2 launch nav_loc_navigation navigation.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nav_loc_navigation')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params,
                'autostart': 'true',
                'use_composition': 'False',
            }.items(),
        ),
        Node(
            package='nav_loc_navigation',
            executable='cmd_vel_to_sport',
            name='cmd_vel_to_sport',
            output='screen',
        ),
    ]

    return LaunchDescription(args + nodes)
