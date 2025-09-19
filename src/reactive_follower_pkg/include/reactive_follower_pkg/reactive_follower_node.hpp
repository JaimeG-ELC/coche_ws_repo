#ifndef REACTIVE_FOLLOWER_NODE_HPP_
#define REACTIVE_FOLLOWER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <interfaces_pkg/msg/goal_point.hpp>
#include <vector>
#include <utility>
#include <memory>
#include <limits>
#include <cmath>
#include <chrono>

// Added required TF2 and geometry messages includes
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class ReactiveFollowerNode : public rclcpp::Node {
public:
    ReactiveFollowerNode();

    struct Gap {
        size_t start;
        size_t end;

        Gap() : start(0), end(0){}
        Gap(size_t start, size_t end)
            : start(start), end(end){}
    };

private:
    // ROS Parameters
    std::string lidarscan_topic;
    std::string goalpoint_topic;
    std::string drive_topic;
    std::string laser_frame;
    std::string car_frame;  // first declaration kept

    double lidar_angle;
    double lidar_angle_front_car;
    int lidar_scans;  // renamed from lidar_scans to match use

    double max_speed;
    double min_speed;
    double max_steering_angle;
    int bubble_radius;
    double processed_angle;
    double safety_distance_min;
    double safety_distance_threshold;
    double safety_distance_gain;
    double vehicle_width;

    double max_lidar_distance;
    double weight_speed;
    double weight_steering;

    // Scan indices and angles
    size_t start_index;
    size_t end_index;
    double start_angle;
    double end_angle;
    size_t gp_index;  // renamed from pg_index to match use
    double safety_distance;
    size_t min_gap_size;
    double lidar_angle_rad;
    double processed_angle_rad;
    double lidar_angle_front_car_rad;
    double max_steering_angle_rad;


    // Transform handling
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::TransformStamped current_transform_;

    // ROS communication
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<interfaces_pkg::msg::GoalPoint>::SharedPtr goal_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;  
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_publisher_;


    // Additional message storage members
    sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_msg_;
    interfaces_pkg::msg::GoalPoint::ConstSharedPtr goal_msg_;

    // LiDAR processing methods

    void preprocess_lidar(std::vector<float> &ranges);
    int find_closest_point(const std::vector<float> &ranges);
    void eliminate_bubble(std::vector<float> &ranges, int closest_idx, float bubble_radius);
    void publishLaser(const std::vector<float> &ranges);


    double calculate_safety_distance(double speed);
    size_t calculate_min_gap_size(double safety_distance);
    size_t point_to_lidar_index();

    std::pair<size_t, size_t> find_gap(const std::vector<float>& ranges, size_t min_gap);
    bool gp_in_gaps(const std::vector<Gap>& gaps, const std::vector<float> &ranges);
    std::pair<double, double> alternative_commands(const std::pair<size_t, size_t>& biggest_gap, const std::vector<float>& ranges);
    
    // Callback
    void goal_callback(const interfaces_pkg::msg::GoalPoint::ConstSharedPtr msg);
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
};

#endif // REACTIVE_FOLLOWER_NODE_HPP_