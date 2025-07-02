# slam_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', '/root/coche_ws/install/f1tenth_stack/share/f1tenth_stack/config',
                '-configuration_basename', 'cartographer.lua',
            ]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            arguments=[
                '-resolution', '0.05',
                '-publish_period_sec', '1.0'
            ]
        )
    ])