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

    int bubble_radius;
    double max_speed;
    double min_speed;
    double lidar_angle;
    double max_lidar_distance;
    double weight_speed;
    double weight_steering;

    // Scan indices and angles
    size_t start_index;
    size_t end_index;
    double start_angle;
    double end_angle;
    int gp_index;  // renamed from pg_index to match use
    double safety_distance;
    size_t min_gap_size;

    // Transform handling
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::TransformStamped current_transform_;

    // ROS communication
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<interfaces_pkg::msg::GoalPoint>::SharedPtr goal_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

    // Additional message storage members
    sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_msg_;
    interfaces_pkg::msg::GoalPoint::ConstSharedPtr goal_msg_;

    // LiDAR processing methods

    void preprocess_lidar(std::vector<float> &ranges);
    size_t find_closest_point(const std::vector<float> &ranges);
    void eliminate_bubble(std::vector<float> &ranges, size_t closest_idx, float bubble_radius);

    double calculate_safety_distance(double speed);
    size_t calculate_min_gap_size(double safety_distance);
    int point_to_lidar_index();

    std::vector<Gap> find_gaps(const std::vector<float>& ranges, size_t min_gap, double safety_distance);
    bool gp_in_gaps(const std::vector<Gap>& gaps);
    std::pair<double, double> alternative_commands(const std::vector<Gap>& gaps, const std::vector<float>& ranges, int gp_index);

    // Callback
    void goal_callback(const interfaces_pkg::msg::GoalPoint::ConstSharedPtr msg);
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
};

#endif // REACTIVE_FOLLOWER_NODE_HPP_