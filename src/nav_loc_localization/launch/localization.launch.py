"""
localization.launch.py — bring up a connected TF tree + /front/scan for the Go2.

In cyclonedds mode go2_robot_sdk's driver is a no-op (its lowstate/odom/pose
callbacks just `pass`), so nothing publishes the URDF, /joint_states or the
odom->base_link TF. RViz then shows no robot and "Frame [odom] does not exist".

This launch owns everything needed for a fully connected TF tree + RobotModel
+ /front/scan, all sourced from what the Go2 main board publishes natively over the
wired CycloneDDS link:

  robot_state_publisher     go2.urdf            -> /robot_description + /tf_static
  joint_state_bridge        /lowstate           -> /joint_states
  odom_tf_broadcaster       /utlidar/robot_odom -> TF odom -> base_link
  odom_relay                /utlidar/robot_odom -> /odometry/filtered (RELIABLE)
  cloud_restamp             /utlidar/cloud_deskewed (robot clock)
                                                -> /utlidar/cloud_deskewed_pcstamp (PC clock)
  pointcloud_to_laserscan   /utlidar/cloud_deskewed_pcstamp -> /front/scan

Run with:
    ros2 launch nav_loc_localization localization.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nav_loc_localization')
    urdf_path = os.path.join(pkg_share, 'urdf', 'go2.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('joint_rate_hz', default_value='50.0',
                              description='Throttle /joint_states output rate'),
    ]

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    joint_state_bridge = Node(
        package='nav_loc_localization',
        executable='joint_state_bridge',
        name='nav_loc_joint_state_bridge',
        output='screen',
        parameters=[{
            'publish_rate_hz': LaunchConfiguration('joint_rate_hz'),
            'use_sim_time': use_sim_time,
        }],
    )

    odom_tf_broadcaster = Node(
        package='nav_loc_localization',
        executable='odom_tf_broadcaster',
        name='nav_loc_odom_tf_broadcaster',
        output='screen',
        parameters=[{
            'odom_topic': '/utlidar/robot_odom',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'use_sim_time': use_sim_time,
        }],
    )

    # Relay the Go2's native odom to /odometry/filtered (RELIABLE) so consumers
    # written for a Jackal (dynamics_planner_nav) connect without QoS mismatch.
    odom_relay = Node(
        package='nav_loc_localization',
        executable='odom_relay',
        name='nav_loc_odom_relay',
        output='screen',
        parameters=[{
            'in_topic': '/utlidar/robot_odom',
            'out_topic': '/odometry/filtered',
            'use_sim_time': use_sim_time,
        }],
    )

    cloud_restamp = Node(
        package='nav_loc_localization',
        executable='cloud_restamp',
        name='nav_loc_cloud_restamp',
        output='screen',
        parameters=[{
            'in_topic': '/utlidar/cloud_deskewed',
            'out_topic': '/utlidar/cloud_deskewed_pcstamp',
            'reliability': 'best_effort',
            'use_sim_time': use_sim_time,
        }],
    )

    # Re-stamp the HESAI XT16 cloud with the PC clock too. The driver publishes
    # /lidar_points RELIABLE on the robot clock; RViz's "HESAI XT16" display
    # subscribes to /lidar_points_pcstamp (BEST_EFFORT). Without this node that
    # topic has no publisher and the lidar never shows up.
    hesai_restamp = Node(
        package='nav_loc_localization',
        executable='cloud_restamp',
        name='nav_loc_hesai_restamp',
        output='screen',
        parameters=[{
            'in_topic': '/lidar_points',
            'out_topic': '/lidar_points_pcstamp',
            'reliability': 'reliable',
            'use_sim_time': use_sim_time,
        }],
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            # /scan is built from the HESAI XT16 (16-beam, longer range/denser)
            # rather than the Go2's onboard L1 lidar. Height filtering below is
            # done in target_frame=base_link, so it is independent of which
            # lidar feeds in; the base_link->hesai_lidar static TF (pure yaw,
            # level) makes the projection correct.
            ('cloud_in', '/lidar_points_pcstamp'),
            ('scan', '/front/scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.1,
            'min_height': 0.2,
            'max_height': 1.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.2,
            'range_max': 30.0,
            'use_inf': True,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription(args + [
        robot_state_publisher,
        joint_state_bridge,
        odom_tf_broadcaster,
        odom_relay,
        cloud_restamp,
        hesai_restamp,
        pointcloud_to_laserscan,
    ])
