"""Launch pp2_to_livox_node (simulation)

Starts the converter node in the simulation package so the driver package remains unchanged.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic_arg = DeclareLaunchArgument(
        'input_topic', default_value='/points', description='Input PointCloud2 topic')

    input_topic = LaunchConfiguration('input_topic')

    pp2_node = Node(
        package='sentry_velocity',
        executable='pp2_to_livox_node',
        name='pp2_to_livox',
        output='screen',
        parameters=[{'input_topic': input_topic}],
    )

    return LaunchDescription([
        input_topic_arg,
        pp2_node,
    ])
