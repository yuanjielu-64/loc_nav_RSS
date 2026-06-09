"""
bringup.launch.py — top-level entry point.

Includes the localization and navigation launch files, and optionally fires up
rviz2 with a preset config. Edit the LaunchArguments below to turn pieces on/off.

Run with:
    ros2 launch nav_loc_bringup bringup.launch.py
    ros2 launch nav_loc_bringup bringup.launch.py rviz:=false  # headless
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nav_loc_bringup')
    loc_share = get_package_share_directory('nav_loc_localization')
    nav_share = get_package_share_directory('nav_loc_navigation')

    rviz_config = os.path.join(pkg_share, 'rviz', 'nav_loc.rviz')

    args = [
        DeclareLaunchArgument('localization', default_value='true',
                              description='Start localization stack'),
        DeclareLaunchArgument('navigation',   default_value='true',
                              description='Start navigation stack'),
        DeclareLaunchArgument('rviz',         default_value='true',
                              description='Start rviz2 with preset config'),
    ]

    include_loc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(loc_share, 'launch', 'localization.launch.py')),
        condition=IfCondition(LaunchConfiguration('localization')),
    )

    include_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'navigation.launch.py')),
        condition=IfCondition(LaunchConfiguration('navigation')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + [include_loc, include_nav, rviz])
