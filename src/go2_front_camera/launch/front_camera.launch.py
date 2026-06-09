"""Launch the Go2 front camera bridge (videohub poll -> Image)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('poll_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('frame_id', default_value='front_camera'),
        DeclareLaunchArgument('publish_decoded', default_value='true'),
        Node(
            package='go2_front_camera',
            executable='front_camera_node',
            name='go2_front_camera',
            output='screen',
            parameters=[{
                'poll_rate_hz': LaunchConfiguration('poll_rate_hz'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_decoded': LaunchConfiguration('publish_decoded'),
            }],
        ),
    ])
