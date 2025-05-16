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

    pure_pursuit_node = Node(
        package="pure_pursuit",
        executable="pure_pursuit_node.py",
        parameters=[
            {"REAL_ENVIRONMENT": False},
            {"CSV_NAME": "Oschersleben_map"},				# Name of the CSV file used for racetrack waypoints.
            {"WAYPOINTS_DIRECTION": True},				# Direction of waypoints following (True = Ascending Order)
            {"MAX_STEER": 0.4},					# Maximum steering angle of your vehicle
            {"MIN_STEER": -0.4},					# Minimum steering angle of your vehicle
            {"MAX_SPEED": 8.0},					# Maximum Speed of your vehicle
            {"G": 0.5},                  				# Gain of steering, used for smoothing turns
            {"L": 1.0},                   				# Lookahead distance of Pure Pursuit
        ],
        output="screen"                                     		          
    )
    
    ld.add_action(gym_bridge_launch)                       
    ld.add_action(pure_pursuit_node)                        

    return ld
