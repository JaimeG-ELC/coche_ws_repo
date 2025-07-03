from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package',
            executable='imu_tf_broadcaster',
            name='imu_tf_broadcaster',
            output='screen'
        )
    ])
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=[
                '0.1', '0.05', '0.1',      # x, y, z (translation)
                '0', '0', '0',               # roll, pitch, yaw (rotation in radians)
                'imu_link', 'base_link'     # parent_frame_id, child_frame_id
            ],
            output='screen'
        )