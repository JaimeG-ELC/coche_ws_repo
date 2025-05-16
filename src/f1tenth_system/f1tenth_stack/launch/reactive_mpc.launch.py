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
    
  mpc_node = Node(
  	package='mpc',
        executable='mpc_reactive.py',
        name='reactive_mpc_controller',
        output='screen',
        parameters=[
          
               {"real_environment": False},          
               {"csv_file": "Oschersleben_map"},      # CSV file name

               {"input_cost_accel": 0.01},            # Cost for acceleration
               {"input_cost_steering": 100.0},        # Cost for steering
               
               {"input_rate_cost_accel": 0.01},       # Rate cost for acceleration
               {"input_rate_cost_steering": 100.0},   # Rate cost for steering
               
               {"state_cost_x": 200.0},               # State Error cost for x
               {"state_cost_y": 200.0},               # State Error cost for y
               {"state_cost_v": 15.0},                # State Error cost for velocity
               {"state_cost_yaw": 0.0},               # State Error cost for yaw
               
               {"final_state_cost_x": 200.0},         # Final State Error cost for x
               {"final_state_cost_y": 200.0},         # Final State Error cost for y
               {"final_state_cost_v": 15.0},          # Final State Error cost for velocity
               {"final_state_cost_yaw": 0.0},         # Final State Error cost for yaw
               
               {"horizon": 6},                        # Horizon Tk
               {"search_idx": 20},                    # Search index
               {"dt": 0.1},                           # Time step dt
               {"step_length": 0.03},                 # Step length

               {"veh_length": 0.58},                  # Your vehicle length (cm)
               {"veh_width": 0.31},                   # Your vehicle width  (cm)
               {"wheelbase": 0.33},                   # Your vehicle wheelbase length (cm)
               {"min_steer": -0.4189},                # Your vehicle minimum steering angle (rad)
               {"max_steer": 0.4189},                 # Your vehicle maximum steering angle (rad)
               {"max_steer_rate": 3.142},             # Your vehicle maximum steering rate (rad/s)
               {"max_speed": 4.0},                    # Your vehicle maximum speed (m/s)
               {"min_speed": 0.0},                    # Your vehicle minimum speed (m/s)
               {"max_accel": 1.0},                     # Your vehicle maximum acceleration (m/s^2)
               
               {"downsampling": 10},		# Downsample factor for lidar data
               {"max_sight": 3.0},			# Maximum range of the lidar sensor
               {"gap_threshold": 15.0},		# Minimum gap length to trigger avoidance
        ]
  )
        
  ld.add_action(gym_bridge_launch)                        
  ld.add_action(mpc_node)     

  return ld
