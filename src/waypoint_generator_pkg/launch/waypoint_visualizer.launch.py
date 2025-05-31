from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('waypoint_generator_pkg'),
        'config',
        'waypoint_visualizer_params.yaml'
    )

    config_la = DeclareLaunchArgument(
        'config',
        default_value=config,
        description='Path to config file'
    )

    visualizer_node = Node(
        package='waypoint_generator_pkg',
        executable='waypoint_visualizer_node',
        name='waypoint_visualizer',
        parameters=[LaunchConfiguration('config')],
        output='screen'
    )

    return LaunchDescription([
        config_la,
        visualizer_node
    ])
