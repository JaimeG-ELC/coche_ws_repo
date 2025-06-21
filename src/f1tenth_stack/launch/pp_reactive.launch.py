from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_f1tenth = get_package_share_directory('f1tenth_stack')
    pkg_pure_pursuit = get_package_share_directory('pure_pursuit_pkg')
    pkg_reactive = get_package_share_directory('reactive_follower_pkg')

    # First launch bringup to establish odom->base_link->laser
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_f1tenth, 'launch', 'bringup_pf.launch.py')
        )
    )

    # Then launch localization to add map->odom
    localize_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_f1tenth, 'launch', 'localize.launch.py')
        )
    )

    # Then launch localization to add map->odom
    pp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_pure_pursuit, 'launch', 'pure_pursuit.launch.py')
        )
    )

    # Then launch localization to add map->odom
    reactive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_reactive, 'launch', 'reactive_follower.launch.py')
        )
    )
    
    return LaunchDescription([
        bringup_launch,
        localize_launch,
        pp_launch
    ])
