#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class ReactiveGapNavigator(Node):
    """
    ROS2 node implementing a reactive gap-following strategy.
    It processes LiDAR scans to determine a safe driving gap and publishes corresponding drive commands.
    """
    def __init__(self):

        super().__init__('gap_navigator')

        # Declare all ROS2 parameters required by this node.
        self.declare_custom_parameters()

        # Parameters for LiDAR preprocessing and gap following.
        self.real_environment = self.get_parameter("REAL_ENVIRONMENT").value
        self.downsample_factor = self.get_parameter("DOWNSAMPLING").value 
        self.maximum_range = float(self.get_parameter("MAXIMUM_RANGE").value)            
        self.safe_gap_distance = float(self.get_parameter("GAP_DISTANCE").value)
        self.max_steer=float(self.get_parameter("MAX_STEER").value)       
        self.min_steer=float(self.get_parameter("MIN_STEER").value)
        self.speed=float(self.get_parameter("SPEED").value)

        # Define topic names for the LiDAR scan and drive commands.
        lidar_topic = '/scan'
        drive_topic = '/drive'
        gap_vis_topic = '/gap_visualization'

        # Subscribe to LiDAR scan messages.
        self.lidar_subscriber = self.create_subscription(LaserScan, lidar_topic, self.scan_callback, 10)
        
        # Publisher for sending drive commands.
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.drive_command = AckermannDriveStamped()

        # Visualization publishers for gap visualization.
        self.gap_marker_pub = self.create_publisher(MarkerArray, gap_vis_topic, 1)
        
    def scan_callback(self, scan_msg):
        """
        Callback function that processes each LiDAR scan.
        It downsamplesthe data, finds the largest gap,
        selects a target point within that gap, computes the steering angle,
        and then publishes a drive command.
        """
        # Extract a subset of the LiDAR scan (indices 180 to 899).
        self.raw_scan = np.array(scan_msg.ranges[180:899])
        
        # Downsample and clip the LiDAR data.
        self.proc_scan = self.preprocess_scan(self.raw_scan)

        # Identify the largest safe gap.
        gap_start, gap_end = self.find_largest_gap(self.proc_scan)
        #print("Largest Gap Index:", gap_start, "-", gap_end)

        # Select the best target point within the gap.
        target_idx = (gap_start + gap_end)/2
        #print("Selected target point index:", target_idx)

        # Convert the target index to a steering angle. The conversion maps the index to an angle (in radians). The index is cliped to physically possible values.
        steering_angle = np.deg2rad(target_idx * self.downsample_factor / 4.0 - 90.0)
        steering_angle = np.clip(steering_angle, self.min_steer, self.max_steer)
        #print("Steering angle (radians):", steering_angle)

        velocity = self.speed
        #print("Velocity:", velocity)

        # Update the drive command message.
        self.drive_command.drive.steering_angle = steering_angle
        self.drive_command.drive.speed = velocity

        # Publish the drive command.
        self.drive_publisher.publish(self.drive_command)

        # Visualize the gap
        self.visualize_gap(gap_start, gap_end)


    def declare_custom_parameters(self):
        """
        Declare all ROS2 parameters required by this node.
        """
        self.declare_parameter("REAL_ENVIRONMENT")
        self.declare_parameter("DOWNSAMPLING")
        self.declare_parameter("MAXIMUM_RANGE")
        self.declare_parameter("GAP_DISTANCE")
        self.declare_parameter("MAX_STEER")
        self.declare_parameter("MIN_STEER")
        self.declare_parameter("SPEED")


    def preprocess_scan(self, raw_ranges):
        """
        Downsamples and clips the LiDAR scan array.
        Each downsampled value is the mean over a window defined by downsample_factor.
        Values are then clipped to not exceed maximum_range.

        :param raw_ranges: Array of LiDAR distance values.
        :return: Downsampled and clipped LiDAR ranges.
        """
        num_samples = int(720 / self.downsample_factor)
        downsampled = np.zeros(num_samples)
        
        for idx in range(num_samples):
            start = idx * self.downsample_factor
            end = (idx + 1) * self.downsample_factor
            window = raw_ranges[start:end]
            downsampled[idx] = sum(window) / self.downsample_factor
        
        downsampled = np.clip(downsampled, 0.0, self.maximum_range)
        return downsampled

    def find_largest_gap(self, processed_ranges):
        """
        Identifies the largest continuous gap in the processed LiDAR data that is considered safe.
        
        :param processed_ranges: Array of processed LiDAR range values.
        :return: Tuple (start_index, end_index) of the largest safe gap.
        """
        longest_gap = 0
        current_gap = 0
        gap_start = 0
        gap_end = 0

        for idx in range(len(processed_ranges)):
            if processed_ranges[idx] > self.safe_gap_distance:
                current_gap += 1
                if current_gap > longest_gap:
                    longest_gap = current_gap
                    gap_end = idx + 1  # End index (exclusive)
                    gap_start = gap_end - longest_gap
            else:
                current_gap = 0

        return gap_start, gap_end
    
    def visualize_gap(self, gap_start, gap_end):
        """
        Visualizes the start and end boundaries of the detected gap as a "fan".
        Creates two line strip markers (one for each boundary) and publishes them as a MarkerArray.
        """
        marker_arr = MarkerArray()
        marker_arr.markers = []

        # Start boundary marker.
        start_angle = np.deg2rad(180 / len(self.proc_scan) * gap_start)
        start_x = np.sin(start_angle) * self.proc_scan[gap_start]
        start_y = -np.cos(start_angle) * self.proc_scan[gap_start]

        start_marker = Marker()
        start_marker.header.frame_id = 'laser' if self.real_environment else 'ego_racecar/base_link'
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
        end_angle = np.deg2rad(180 / len(self.proc_scan) * gap_end)
        end_x = np.sin(end_angle) * self.proc_scan[gap_end]
        end_y = -np.cos(end_angle) * self.proc_scan[gap_end]

        end_marker = Marker()
        end_marker.header.frame_id = 'laser' if self.real_environment else 'ego_racecar/base_link'
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

        self.gap_marker_pub.publish(marker_arr)


def main(args=None):
    rclpy.init(args=args)
    print("Reactive Gap Navigator Initialized")
    gap_navigator = ReactiveGapNavigator()
    rclpy.spin(gap_navigator)
    gap_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
