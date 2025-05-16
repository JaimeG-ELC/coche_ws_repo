#!/usr/bin/env python3

import numpy as np
from scipy.spatial import distance, transform
import os
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, Pose, PoseArray

class PurePursuitDriver(Node):
    """
    A ROS2 node implementing the Pure Pursuit algorithm for vehicle control.
    This node subscribes to pose and gap following points, computes the steering and speed commands,
    and publishes drive commands along with several visualization markers.
    """
    def __init__(self):
        super().__init__('pure_pursuit_driver')

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

        # Define topics for drive commands, odometry, and visualization.
        drive_cmd_topic = '/drive'
        odom_topic = '/pf/viz/inferred_pose' if self.real else '/ego_racecar/odom'
        vis_marker_topic = '/waypoints'
        pp_pose_topic = '/pp_point'
        gf_point_topic = '/gf_point'

        # Initialize the waypoints and reference speeds from the provided CSV file from TUM RL-Opt.
        self.csv_path=os.path.abspath(os.path.join('src', 'csv_data'))
        csv_data = np.loadtxt(os.path.join(self.csv_path, self.csv_filename + '.csv'), delimiter=',', skiprows=0)
        self.waypoints = csv_data[:, 1:3] # Extract x and y coordinates.
        self.num_waypoints = self.waypoints.shape[0]
        self.ref_speeds = csv_data[:, 5] # Extract reference speeds.

        # Subscribe to pose/odometry messages.
        self.pose_subscriber = self.create_subscription(PoseStamped if self.real else Odometry, odom_topic, self.on_pose_received, 1)

        # Publisher for drive commands.
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, drive_cmd_topic, 1)
        self.drive_command = AckermannDriveStamped()

        # Publish goal poses (in the vehicle frame) to the gap following node.
        self.pp_pose_publisher = self.create_publisher(PoseArray, pp_pose_topic, 1)

        # Subscribe to the resultant gap following point.
        self.gf_point_subscriber = self.create_subscription(Point, gf_point_topic, self.on_gap_following, 1)

        # Publisher for visualization markers.
        self.vis_marker_publisher = self.create_publisher(MarkerArray, vis_marker_topic, 1)
        self.marker_array = MarkerArray()

        # Initialize all visualization markers.
        self.initialize_markers()

    def setup_parameters(self):
        """
        Declares all ROS2 parameters used by this node provided in launch file.
        """
        self.real = self.declare_parameter("REAL_ENVIRONMENT")
        self.csv_filename = self.declare_parameter("CSV_NAME")
        self.direction = self.declare_parameter("WAYPOINTS_DIRECTION")
        self.max_steer=self.declare_parameter("MAX_STEER")
        self.min_steer=self.declare_parameter("MIN_STEER")
        self.max_speed=self.declare_parameter("MAX_SPEED")
        self.L = self.declare_parameter("L")
        self.G = self.declare_parameter("G")

    def on_pose_received(self, pose_msg):
        """
        Callback for incoming pose (or odometry) messages.
        Processes the current vehicle state, finds the nearest and target waypoints,
        and publishes both the drive command and visualization markers.
        """
        # Extract current position based on operation mode.
        if self.real:
            self.current_x = pose_msg.pose.position.x
            self.current_y = pose_msg.pose.position.y
        else:
            self.current_x = pose_msg.pose.pose.position.x
            self.current_y = pose_msg.pose.pose.position.y
        self.current_position = np.array([self.current_x, self.current_y]).reshape((1, 2))

        # Convert quaternion orientation to a rotation matrix.
        if self.real:
            orientation = pose_msg.pose.orientation
        else:
            orientation = pose_msg.pose.pose.orientation
        quat_vals = [orientation.x, orientation.y, orientation.z, orientation.w]
        rot_obj = transform.Rotation.from_quat(quat_vals)
        self.rotation_matrix = rot_obj.as_matrix()

        # Calculate Euclidean distances from the current position to all waypoints.
        self.dist_to_waypoints = distance.cdist(self.current_position, self.waypoints, 'euclidean').reshape((self.num_waypoints))

        # Identify the nearest waypoint.
        self.nearest_index = np.argmin(self.dist_to_waypoints)
        self.nearest_waypoint = self.waypoints[self.nearest_index]

        # Determine the target waypoint beyond the lookahead distance.
        target_waypoint = self.find_target_waypoint(self.L)

        # Transform the target waypoint vehicle coordinate frame.
        transformed_target = self.transform_target_point(target_waypoint)

        # Create a PoseArray message to send the transformed target, nearest, and current poses.
        pose_array_msg = PoseArray()

        # Append target pose.
        target_pose = Pose()
        target_pose.position.x = transformed_target[0]
        target_pose.position.y = transformed_target[1]
        pose_array_msg.poses.append(target_pose)

        # Append nearest waypoint pose.
        nearest_pose = Pose()
        nearest_pose.position.x = self.nearest_waypoint[0]
        nearest_pose.position.y = self.nearest_waypoint[1]
        pose_array_msg.poses.append(nearest_pose)

        # Append current vehicle pose.
        current_pose = Pose()
        current_pose.position.x = self.current_x
        current_pose.position.y = self.current_y
        pose_array_msg.poses.append(current_pose)

        # Publish the pose array for gap following.
        self.pp_pose_publisher.publish(pose_array_msg)

        # Update visualization markers for the nearest and target waypoints.
        self.closest_marker.points = [Point(x=self.nearest_waypoint[0], y=self.nearest_waypoint[1], z=0.0)]
        self.target_marker.points = [Point(x=target_waypoint[0], y=target_waypoint[1], z=0.0)]
        self.marker_array.markers = [self.waypoints_marker, self.closest_marker, self.target_marker]
        self.vis_marker_publisher.publish(self.marker_array)

    def on_gap_following(self, gf_point_msg):
        """
        Callback for gap following point messages.
        Uses the gap following point to compute a new steering angle and speed,
        then publishes the drive command and its visualization.
        """
        # Extract gap following point coordinates.
        gf_x = gf_point_msg.x
        gf_y = gf_point_msg.y

        # Interpret the z-value as a boolean flag for obstacle detection.
        obstacle_detected = bool(gf_point_msg.z)
        
        # Update the gap following visualization marker.
        self.gf_point_marker.points = [Point(x=gf_x, y=gf_y, z=0.2)]

        # Calculate steering command based on the lateral gap following offset. Clamp it to physical limits.
        steering_cmd = self.G * (2 * gf_y / self.L**2)
        self.drive_command.drive.steering_angle = np.clip(steering_cmd, -0.4, 0.4)
        #print("Steering Angle:", self.drive_command.drive.steering_angle)

        # Determine drive speed based on reference speed and obstacle presence.
        base_speed = self.ref_speeds[self.nearest_index]
        self.drive_command.drive.speed = (0.5 if obstacle_detected else 1.0) * base_speed
        #print("Speed:", self.drive_command.drive.speed)
        
        # Publish the computed drive command.
        self.drive_publisher.publish(self.drive_command)

    def find_target_waypoint(self, min_distance):
        """
        Iterates through waypoints to find the first one that lies beyond the specified lookahead distance.
        The search direction is determined by the 'ascending_waypoints' parameter.
        """
        idx = self.nearest_index
        curr_dist = self.dist_to_waypoints[idx]
        while curr_dist < min_distance:
            if self.direction:
                idx += 1
                if idx >= len(self.waypoints):
                    idx = 0
            else:
                idx -= 1
                if idx < 0:
                    idx = len(self.waypoints) - 1
            curr_dist = self.dist_to_waypoints[idx]
        return self.waypoints[idx]

    def transform_target_point(self, target_point):
        """
        Applies a homogeneous transformation to convert a target point
        from the global coordinate frame to the vehicle's coordinate frame.
        """
        # Construct a 4x4 transformation matrix.
        T = np.zeros((4, 4))
        T[0:3, 0:3] = np.linalg.inv(self.rotation_matrix)
        T[0, 3] = self.current_x
        T[1, 3] = self.current_y
        T[3, 3] = 1.0
        
        # Compute the difference vector from the current position to the target point.
        vec_diff = target_point - self.current_position
        transformed = (T @ np.array((vec_diff[0, 0], vec_diff[0, 1], 0, 0))).reshape((4))
        return transformed

    def initialize_markers(self):
        """
        Sets up the markers used for visualization, including markers for
        waypoints, the target waypoint, the nearest waypoint, and the gap following point.
        """
        # Marker for all waypoints.
        self.waypoints_marker = Marker()
        self.waypoints_marker.header.frame_id = 'map'
        self.waypoints_marker.type = Marker.POINTS
        self.waypoints_marker.color.b = 1.0
        self.waypoints_marker.color.a = 1.0
        self.waypoints_marker.scale.x = 0.05
        self.waypoints_marker.scale.y = 0.05
        self.waypoints_marker.id = 0
        self.waypoints_marker.points = [Point(x=pt[0], y=pt[1], z=0.0) for pt in self.waypoints]

        # Marker for the target waypoint.
        self.target_marker = Marker()
        self.target_marker.header.frame_id = 'map'
        self.target_marker.type = Marker.POINTS
        self.target_marker.color.r = 0.255
        self.target_marker.color.g = 0.255
        self.target_marker.color.b = 0.0
        self.target_marker.color.a = 1.0
        self.target_marker.scale.x = 0.1
        self.target_marker.scale.y = 0.1
        self.target_marker.id = 1

        # Marker for the nearest waypoint.
        self.closest_marker = Marker()
        self.closest_marker.header.frame_id = 'map'
        self.closest_marker.type = Marker.SPHERE_LIST
        self.closest_marker.color.r = 0.75
        self.closest_marker.color.a = 0.4
        self.closest_marker.scale.x = 0.0
        self.closest_marker.scale.y = 0.0
        self.closest_marker.scale.z = 0.0
        self.closest_marker.id = 2

        # Marker for the gap following point.
        self.gf_point_marker = Marker()
        self.gf_point_marker.header.frame_id = 'laser' if self.real else 'ego_racecar/base_link'
        self.gf_point_marker.type = Marker.POINTS
        self.gf_point_marker.color.g = 0.75
        self.gf_point_marker.color.a = 1.0
        self.gf_point_marker.scale.x = 0.2
        self.gf_point_marker.scale.y = 0.2
        self.gf_point_marker.id = 3


def main(args=None):

    rclpy.init(args=args)
    driver_node = PurePursuitDriver()
    rclpy.spin(driver_node)
    driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
