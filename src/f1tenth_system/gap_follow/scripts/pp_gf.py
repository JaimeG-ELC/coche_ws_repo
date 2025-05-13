#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry


class ReactiveWallFollower(Node):
    """
    ROS2 node for reactive wall following using gap detection.
    This node processes LiDAR scans, odometry, and pure pursuit points,
    then publishes a modified goal point (with an obstacle flag) along with visualization markers.
    """
    def __init__(self):
        super().__init__('reactive_wall_follower')

        # Declare all ROS2 parameters required by this node.
        self.declare_custom_parameters()

        # Load parameters for gap following
        self.is_simulation = self.get_parameter("REAL_ENVIRONMENT").value        
        self.ds_gap = self.get_parameter("DOWNSAMPLING").value 
        self.max_view_range = float(self.get_parameter("MAXIMUM_RANGE").value)
        self.gap_distance_offset = float(self.get_parameter("GAP_DISTANCE_OFFSET").value) 
        self.gap_distance_gain = float(self.get_parameter("GAP_DISTANCE_GAIN").value)     
        self.pp_confidence = float(self.get_parameter("PP_RATIO").value)
        self.lookahead = float(self.get_parameter("L").value)
        self.obstacle_distance = float(self.get_parameter("OBS_DIST").value)

        # Define topics for subscriptions and publications.
        lidar_topic = '/scan'
        pp_topic = '/pp_point'
        gf_topic = '/gf_point'
        gf_vis_topic = '/vis_gf_marker'
        gap_vis_topic = '/vis_gap_marker'
        odom_topic = '/pf/viz/inferred_pose' if self.is_simulation else '/ego_racecar/odom'

        # Subscribe to LiDAR scans.
        self.lidar_sub = self.create_subscription(LaserScan, lidar_topic, self.lidar_callback, 10)
        
        # Subscribe to odometry (PoseStamped in sim, Odometry on real hardware).
        self.odom_sub = self.create_subscription(PoseStamped if self.is_simulation else Odometry,odom_topic, self.odometry_callback, 1)

        # Subscribe to pure pursuit points.
        self.pp_sub = self.create_subscription(PoseArray, pp_topic, self.pure_pursuit_callback, 1)

        # Publisher: send modified goal point to the pure pursuit module.
        self.gf_pub = self.create_publisher(Point, gf_topic, 1)

        # Visualization publishers for gap following and gap fan markers.
        self.raw_gf_marker_pub = self.create_publisher(Marker, gf_vis_topic, 1)
        self.gap_fan_marker_pub = self.create_publisher(MarkerArray, gap_vis_topic, 1)
    
    def lidar_callback(self, lidar_msg):
        """
        Callback for LiDAR scan messages.
        Downsamples the scan data and stores the result for use in gap detection.
        """
        # Configure numpy to print full arrays for debugging.
        np.set_printoptions(threshold=np.inf)

        # Extract LiDAR ranges from indices 180 to 899.
        raw = np.array(lidar_msg.ranges[180:899])
        self.processed_ranges = self.downsample_lidar(raw)

    def odometry_callback(self, odom_msg):
        """
        Callback for odometry messages.
        Updates the current speed of the vehicle.
        """
        # Extract the linear velocity (assumes twist.twist.linear is present).
        linear_vel = odom_msg.twist.twist.linear
        self.current_velocity = np.sqrt(linear_vel.x ** 2 + linear_vel.y ** 2)

    def pure_pursuit_callback(self, pp_msg):
        """
        Callback for pure pursuit messages.
        Receives the lookahead point and fuses it with gap detection data to
        compute a modified goal point, which is then published.
        """
        # Extract lookahead point from the first pose in the array.
        la_x = pp_msg.poses[0].position.x
        la_y = pp_msg.poses[0].position.y

        # Find the maximum gap within the processed LiDAR data.
        gap_start, gap_end = self.compute_max_gap(self.processed_ranges)
        gap_start = int(np.clip(gap_start, 0, len(self.processed_ranges) - 1))
        gap_end = int(np.clip(gap_end, 0, len(self.processed_ranges) - 1))
        #self.get_logger().info(f"Gap indices: {gap_start} - {gap_end}")

        # If the detected gap is too narrow, assume an obstacle is present.
        if (gap_end - gap_start) < self.obstacle_distance:
            #self.get_logger().info("Obstacle detected!")
            self.obstacle_flag = 1
            self.visualize_gap(gap_start, gap_end)

            # Choose the best point within the gap (naively the midpoint).
            best_idx = (gap_start + gap_end) / 2
            best_idx = int(np.clip(best_idx, 0, len(self.processed_ranges)))
            self.visualize_raw_gap_point(best_idx)

            # Convert the index into an angle and then to x, y coordinates.
            best_angle = np.deg2rad(180 / len(self.processed_ranges) * best_idx)
            gf_x = np.sin(best_angle) * self.lookahead
            gf_y = -np.cos(best_angle) * self.lookahead

            # Blend the gap-following point with the original pp point according to confidence ratio
            mod_x = gf_x * (1 - self.pp_confidence) + la_x * self.pp_confidence
            mod_y = gf_y * (1 - self.pp_confidence) + la_y * self.pp_confidence
        else:
            #self.get_logger().info("No obstacle detected")
            self.obstacle_flag = 0
            mod_x = la_x
            mod_y = la_y

        # Publish the modified goal point (z carries the obstacle flag).
        goal = Point(x=float(mod_x), y=float(mod_y), z=float(self.obstacle_flag))
        self.gf_pub.publish(goal)


    def declare_custom_parameters(self):
        """
        Declare all ROS2 parameters required by this node.
        """
        self.declare_parameter("REAL_ENVIRONMENT")
        self.declare_parameter("DOWNSAMPLING")
        self.declare_parameter("MAXIMUM_RANGE")
        self.declare_parameter("GAP_DISTANCE_GAIN")
        self.declare_parameter("GAP_DISTANCE_OFFSET")
        self.declare_parameter("L")
        self.declare_parameter("PP_RATIO")
        self.declare_parameter("OBS_DIST")

    def downsample_lidar(self, lidar_ranges):
        """
        Downsamples the LiDAR scan data by averaging over windows defined by ds_gap.
        Clips the downsampled values to the maximum view range.
        """
        num_bins = int(720 / self.ds_gap)
        downsampled = np.zeros(num_bins)
        for i in range(num_bins):
            start = i * self.ds_gap
            end = (i + 1) * self.ds_gap
            downsampled[i] = sum(lidar_ranges[start:end]) / self.ds_gap
        return np.clip(downsampled, 0.0, self.max_view_range)

    def compute_max_gap(self, free_ranges):
        """
        Finds the largest continuous gap in free_ranges that exceeds a dynamic safety threshold.
        The threshold is adjusted based on the current speed.
        Returns the start and end indices of the maximum gap.
        """
        # Dynamically adjust safe gap distance based on speed.
        self.max_gap_safe = 1.2 + self.gap_distance_gain * self.current_velocity
        #self.get_logger().info(f"Gap distance: {self.max_gap_safe:.2f} m")

        longest_streak = 0
        streak = 0
        gap_end = 0
        gap_start = 0

        for i in range(len(free_ranges)):
            if free_ranges[i] > self.max_gap_safe:
                streak += 1
                if streak > longest_streak:
                    longest_streak = streak
                    gap_end = i + 1  # End index is exclusive.
                    gap_start = gap_end - longest_streak
            else:
                streak = 0

        return gap_start, gap_end

    def visualize_raw_gap_point(self, index):
        """
        Visualizes the raw gap-following point as a Marker.
        Converts the index to an angle and computes corresponding coordinates.
        """
        angle = np.deg2rad(180 / len(self.processed_ranges) * index)
        x_coord = np.sin(angle) * self.lookahead
        y_coord = -np.cos(angle) * self.lookahead

        marker = Marker()
        marker.header.frame_id = 'laser' if self.is_simulation else 'ego_racecar/base_link'
        marker.type = Marker.POINTS
        marker.color.r = 0.75
        marker.color.g = 0.75
        marker.color.a = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.id = 6
        marker.points = [Point(x=x_coord, y=y_coord, z=0.2)]
        self.raw_gf_marker_pub.publish(marker)

    def visualize_gap(self, gap_start, gap_end):
        """
        Visualizes the start and end boundaries of the detected gap as a "fan".
        Creates two line strip markers (one for each boundary) and publishes them as a MarkerArray.
        """
        marker_arr = MarkerArray()
        marker_arr.markers = []

        # Start boundary marker.
        start_angle = np.deg2rad(180 / len(self.processed_ranges) * gap_start)
        start_x = np.sin(start_angle) * self.processed_ranges[gap_start]
        start_y = -np.cos(start_angle) * self.processed_ranges[gap_start]

        start_marker = Marker()
        start_marker.header.frame_id = 'laser' if self.is_simulation else 'ego_racecar/base_link'
        start_marker.type = Marker.LINE_STRIP
        start_marker.color.r = 1.0
        start_marker.color.g = 0.0
        start_marker.color.b = 0.0
        start_marker.color.a = 1.0
        start_marker.scale.x = 0.04
        start_marker.id = 4
        start_marker.points = [
            Point(x=start_x, y=start_y, z=0.2),
            Point(x=0.0, y=0.0, z=0.2)
        ]
        marker_arr.markers.append(start_marker)

        # End boundary marker.
        end_angle = np.deg2rad(180 / len(self.processed_ranges) * gap_end)
        end_x = np.sin(end_angle) * self.processed_ranges[gap_end]
        end_y = -np.cos(end_angle) * self.processed_ranges[gap_end]

        end_marker = Marker()
        end_marker.header.frame_id = 'laser' if self.is_simulation else 'ego_racecar/base_link'
        end_marker.type = Marker.LINE_STRIP
        end_marker.color.r = 1.0
        end_marker.color.g = 0.0
        end_marker.color.b = 0.0
        end_marker.color.a = 1.0
        end_marker.scale.x = 0.04
        end_marker.id = 5
        end_marker.points = [
            Point(x=end_x, y=end_y, z=0.2),
            Point(x=0.0, y=0.0, z=0.2)
        ]
        marker_arr.markers.append(end_marker)

        self.gap_fan_marker_pub.publish(marker_arr)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    node = ReactiveWallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
