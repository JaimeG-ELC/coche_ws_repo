# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():

    robot_localization_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'ekf_copy.yaml'
    )

    # Launch arguments
    robot_localization_la = DeclareLaunchArgument(
        'robot_localization_config',
        default_value=robot_localization_config,
        description='Descriptions for robot localization configs'
    )

    ld = LaunchDescription([robot_localization_la])

    robot_localization_local_node = Node(
        package='robot_localization',   
        executable='ekf_node',
        name='ekf_filter_local_node',
        output='screen',
        parameters=[LaunchConfiguration('robot_localization_config')],
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    robot_localization_global_node = Node(
        package='robot_localization',   
        executable='ekf_node',
        name='ekf_filter_global_node',
        output='screen',
        parameters=[LaunchConfiguration('robot_localization_config')],
        remappings=[('odometry/filtered', 'odometry/global')]
    )

    static_tf_base_link_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.09', '-0.02', '0.06', '-3.14', '0.0', '0', 'base_link', 'imu']
    )

    static_tf_baselink_laser_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.36', '0.0', '0.12', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )

    # finalize
    # ld.add_action(static_tf_baselink_laser_node)
    ld.add_action(static_tf_base_link_imu_node)
    ld.add_action(robot_localization_local_node)
    # ld.add_action(robot_localization_global_node)

    return ld