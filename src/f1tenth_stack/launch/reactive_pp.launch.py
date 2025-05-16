from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


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
        executable="pp_gf.py",
        output="screen",
        parameters=[

            {"REAL_ENVIRONMENT":False},
            {"DOWNSAMPLING": 10},                     # Downsampling factor for LIDAR data
            {"MAXIMUM_RANGE": 10.0},                  # Maximum range of the LIDAR
            {"GAP_DISTANCE_OFFSET": 1.2},		# Distance to gaps: Base distance. (Affects the vehicle's angular range of vision)
            {"GAP_DISTANCE_GAIN": 0.2},          	# Distance to gaps: Gain to increase distance according to velocity.
            {"L": 1.0}, 				# Lookahead distance of Pure Pursuit
            {"PP_RATIO": 0.01},			# Ratio to blend pp and gf points. [0-1]
            {"OBS_DIST": 18.0},			# Distance to consider an obstacle.
        ]
    )

    pure_pursuit_node = Node(
        package="pure_pursuit",
        executable="gf_pp.py",
        output="screen",
        parameters=[
            {"REAL_ENVIRONMENT": False},				
            {"CSV_NAME": "Oschersleben_map"},				# Name of the CSV file used for racetrack waypoints
            {"WAYPOINTS_DIRECTION": True},				# Direction of waypoints following (True = Ascending Order)
            {"MAX_STEER": 0.4},					# Maximum steering angle of your vehicle
            {"MIN_STEER": 0.4},					# Minimum steering angle of your vehicle
            {"MAX_SPEED": 8.0},					# Maximum Speed of your vehicle
            {"G": 0.5},                  				# Gain of steering, used for smoothing turns
            {"L": 1.0},                   				# Lookahead distance of Pure Pursuit
        ]                                    		          
    )
    

    ld.add_action(gym_bridge_launch)                        
    ld.add_action(gap_follow_node)                         
    ld.add_action(pure_pursuit_node)                       

    return ld
