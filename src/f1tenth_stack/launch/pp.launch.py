from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_f1tenth = get_package_share_directory('f1tenth_stack')
    pkg_pure_pursuit = get_package_share_directory('pure_pursuit_pkg')
    pkg_reactive = get_package_share_directory('reactive_follower_pkg')

    pp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_pure_pursuit, 'launch', 'pure_pursuit.launch.py')
        )
    )
    
    return LaunchDescription([
        pp_launch
    ])
