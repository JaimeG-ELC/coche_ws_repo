#!/usr/bin/env python3

import numpy as np
from scipy.spatial import distance, transform
import os
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped

class PurePursuitController(Node):
    """
    A ROS2 node implementing a Pure Pursuit controller for a car.
    It subscribes to pose/odometry data, computes the steering angle based on a lookahead distance,
    and publishes drive commands along with visualization markers.
    """
    def __init__(self):

        # Initialize the node with a new name.
        super().__init__('pure_pursuit_controller')

        # Declare necessary parameters.
        self.setup_parameters()

        # Determine operation mode (simulation or real) and waypoint ordering.
        self.real = self.get_parameter("REAL_ENVIRONMENT").value
        self.csv_filename = self.get_parameter("CSV_NAME").value
        self.direction = self.get_parameter("WAYPOINTS_DIRECTION").value
        self.max_steer=float(self.get_parameter("MAX_STEER").value)
        self.min_steer=float(self.get_parameter("MIN_STEER").value)
        self.max_speed=float(self.get_parameter("MAX_SPEED").value)
        self.L = float(self.get_parameter("L").value)
        self.G = float(self.get_parameter("G").value)
        
        # Topic definitions for drive commands, odometry, and visualization.
        drive_topic = '/drive'
        odom_source = '/pf/viz/inferred_pose' if self.real else '/ego_racecar/odom'
        vis_topic = '/visualization_marker_array'

        # Initialize the waypoints and reference speeds from the provided CSV file from TUM RL-Opt.
        self.csv_path=os.path.abspath(os.path.join('src', 'csv_data'))
        csv_data = np.loadtxt(os.path.join(self.csv_path, self.csv_filename + '.csv'), delimiter=',', skiprows=0)
        self.waypoints = csv_data[:, 1:3] # Extract x and y coordinates.
        self.num_waypoints = self.waypoints.shape[0]
        self.ref_speeds = csv_data[:, 5] # Extract reference speeds.

        # Subscribe to the pose topic; type depends on simulation (Odometry) or real mode (PoseStamped).
        self.pose_subscriber = self.create_subscription(PoseStamped if self.real else Odometry, odom_source, self.handle_pose, 1)
        
        # Publisher for drive commands (Ackermann messages).
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 1)
        self.drive_command = AckermannDriveStamped()

        # Publisher for visualization markers.
        self.marker_publisher = self.create_publisher(MarkerArray, vis_topic, 1)
        self.marker_array = MarkerArray()

        # Initialize markers for visualization.
        self.initialize_visualization()


    def setup_parameters(self):
        """
        Declares all ROS2 parameters used by this node.
        """
        self.real = self.declare_parameter("REAL_ENVIRONMENT")
        self.csv_filename = self.declare_parameter("CSV_NAME")
        self.direction = self.declare_parameter("WAYPOINTS_DIRECTION")
        self.max_steer=self.declare_parameter("MAX_STEER")
        self.min_steer=self.declare_parameter("MIN_STEER")
        self.max_speed=self.declare_parameter("MAX_SPEED")
        self.L = self.declare_parameter("L")
        self.G = self.declare_parameter("G")


    def handle_pose(self, msg):
        """
        Callback function to process incoming pose or odometry messages.
        It computes the target waypoint, calculates the required steering,
        and publishes the drive command and visualization markers.
        """
        # Extract the current x and y coordinates.
        if self.real:
            self.current_x = msg.pose.position.x
            self.current_y = msg.pose.position.y
        else:
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
        
        # Create a NumPy array for the current position.
        self.current_position = np.array([self.current_x, self.current_y]).reshape((1, 2))

        # Get the current orientation in quaternion format.
        if self.real:
            quat = msg.pose.orientation
        else:
            quat = msg.pose.pose.orientation
        quat_values = [quat.x, quat.y, quat.z, quat.w]
        
        # Convert quaternion to a rotation matrix.
        R = transform.Rotation.from_quat(quat_values)
        self.rotation_matrix = R.as_matrix()

        # Calculate distances from current position to all waypoints.
        self.distance_array = distance.cdist(self.current_position, self.waypoints, 'euclidean').reshape((self.num_waypoints))

        # Identify the closest waypoint index.
        self.nearest_index = np.argmin(self.distance_array)
        self.nearest_point = self.waypoints[self.nearest_index]

        # Select the target point beyond the lookahead distance.
        target_point = self.find_target_point(self.L)

        # Transform the target point into the vehicle's coordinate frame.
        transformed_target = self.transform_target_point(target_point)
        
        # Compute the steering angle and limit it to physically feasible values.
        lateral_offset = transformed_target[1]
        steering_angle = self.G * (2 * lateral_offset / self.L**2)
        steering_angle = np.clip(steering_angle, self.min_steer, self.max_steer)
        
        # Assign the computed steering angle and speed to the drive command.
        self.drive_command.drive.steering_angle = steering_angle
        self.drive_command.drive.speed = np.clip(self.ref_speeds[self.nearest_index],0,self.max_speed)

        # Publish the drive command.
        self.drive_publisher.publish(self.drive_command)
        print("steering = {}, speed = {}".format(round(steering_angle, 2), round(self.drive_command.drive.speed, 2)))

        # Update visualization markers with the target and closest waypoint positions.
        self.target_marker.points = [Point(x=target_point[0], y=target_point[1], z=0.0)]
        self.closest_marker.points = [Point(x=self.nearest_point[0], y=self.nearest_point[1], z=0.0)]
        self.marker_array.markers = [self.waypoint_marker, self.target_marker, self.closest_marker]
        self.marker_publisher.publish(self.marker_array)
        
    def find_target_point(self, threshold):
        """
        Finds and returns the first waypoint that is beyond the specified lookahead distance.
        It moves in ascending or descending order based on the controller's configuration.
        """
        index = self.nearest_index
        current_distance = self.distance_array[index]
        
        # Loop until a waypoint is found that exceeds the lookahead distance.
        while current_distance < threshold:
            if self.direction:
                index += 1
                if index >= len(self.waypoints):
                    index = 0
            else:
                index -= 1
                if index < 0:
                    index = len(self.waypoints) - 1
            current_distance = self.distance_array[index]

        return self.waypoints[index]

    def transform_target_point(self, target):
        """
        Transforms the target waypoint from the global coordinate frame
        to the vehicle's coordinate frame using a homogeneous transformation.
        """
        # Create a 4x4 homogeneous transformation matrix.
        H = np.zeros((4, 4))
        # Inverse rotation is used to transform coordinates.
        H[0:3, 0:3] = np.linalg.inv(self.rotation_matrix)
        # Set the translation component.
        H[0, 3] = self.current_x
        H[1, 3] = self.current_y
        H[3, 3] = 1.0
        
        # Compute the vector from the current position to the target.
        vector_diff = target - self.current_position

        # Apply the transformation.
        converted = (H @ np.array((vector_diff[0, 0], vector_diff[0, 1], 0, 0))).reshape((4))
        
        return converted

    def initialize_visualization(self):
        """
        Initializes the markers used for visualizing waypoints,
        the target point, and the closest waypoint.
        """
        # Marker for all waypoints (blue points).
        self.waypoint_marker = Marker()
        self.waypoint_marker.header.frame_id = 'map'
        self.waypoint_marker.type = Marker.POINTS
        self.waypoint_marker.color.b = 1.0
        self.waypoint_marker.color.a = 1.0
        self.waypoint_marker.scale.x = 0.05
        self.waypoint_marker.scale.y = 0.05
        self.waypoint_marker.id = 0
        self.waypoint_marker.points = [Point(x=pt[0], y=pt[1], z=0.0) for pt in self.waypoints]

        # Marker for the target point (yellowish point).
        self.target_marker = Marker()
        self.target_marker.header.frame_id = 'map'
        self.target_marker.type = Marker.POINTS
        self.target_marker.color.r = 0.255
        self.target_marker.color.g = 0.255
        self.target_marker.color.b = 0.0
        self.target_marker.color.a = 1.0
        self.target_marker.scale.x = 0.2
        self.target_marker.scale.y = 0.2
        self.target_marker.id = 1

        # Marker for the closest waypoint (blueish point).
        self.closest_marker = Marker()
        self.closest_marker.header.frame_id = 'map'
        self.closest_marker.type = Marker.POINTS
        self.closest_marker.color.b = 0.75
        self.closest_marker.color.a = 1.0
        self.closest_marker.scale.x = 0.2
        self.closest_marker.scale.y = 0.2
        self.closest_marker.id = 2

def main(args=None):
    """
    Entry point for the Pure Pursuit controller node.
    Initializes the ROS2 node and begins processing.
    """
    rclpy.init(args=args)
    controller = PurePursuitController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
