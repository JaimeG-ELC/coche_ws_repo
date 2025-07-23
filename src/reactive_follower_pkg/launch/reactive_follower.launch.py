from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    reactive_follower_config = os.path.join(
        get_package_share_directory('reactive_follower_pkg'),
        'config',
        'reactive_follower.yaml'
    )

    reactive_follower_la = DeclareLaunchArgument(
        'reactive_follower_config',
        default_value=reactive_follower_config
    )

    reactive_follower_node = Node(
        package="reactive_follower_pkg",
        executable="reactive_follower_node",
        name="reactive_follower_node",
        parameters=[LaunchConfiguration('reactive_follower_config')]
    )

    ld = LaunchDescription([reactive_follower_la])

    ld.add_action(reactive_follower_node)

    return ld