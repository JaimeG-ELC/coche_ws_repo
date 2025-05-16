from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    ld = LaunchDescription()
    
    #simulation
    gym_bridge_launch = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource(
        	[
                os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch'),
                '/gym_bridge_launch.py'
        	]
        )
    )

    gap_follow_node = Node(
        package="gap_follow",
        executable="gap_follow.py",
        parameters=[
            {"REAL_ENVIRONMENT":False},
            {"DOWNSAMPLING": 10},                     # Downsampling factor for LIDAR data
            {"MAXIMUM_RANGE": 10.0},                  # Maximum range of the LIDAR
            {"GAP_DISTANCE": 3},          		# Distance to the gaps analyzed. Affects the vehicle's angular range of vision
            {"MAX_STEER": 0.4},			# Maximum steering angle of your vehicle
            {"MIN_STEER": -0.4},			# Minimum steering angle of your vehicle
            {"SPEED": 6.0},				# Speed of the vehicle
        ],
        output="screen"
  
    )

    ld.add_action(gym_bridge_launch)                        
    ld.add_action(gap_follow_node)                                             

    return ld
