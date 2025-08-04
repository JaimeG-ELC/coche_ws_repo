#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <eigen3/Eigen/Dense>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include "interfaces_pkg/msg/goal_point.hpp"

class PurePursuit : public rclcpp::Node
{
public:
    PurePursuit();

    // Required Structures
    struct PathPoint
    {
        double x, y, v;

        PathPoint() : x(0.0), y(0.0), v(0.0){}
        PathPoint(double x, double y, double v) : x(x), y(y), v(v){};
    };

private:
    // Variables
    int closest_pathpoint = 0;
    int lookahead_point = 0;
    double lookahead_dist = 0.0;
    double steering_angle = 0.0;
    double speed = 0.0;
    int n_pathpoints;
    int start_index = 0;

    PathPoint curr_pose;
    std::vector<PathPoint> pathpoints;

    // Lookahead target in map and local frames
    Eigen::Vector3d v_global = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_local = Eigen::Vector3d::Zero();

    // Cached transform for this cycle
    geometry_msgs::msg::TransformStamped current_transform_;

    // Parameters
    double min_lookahead_dist;
    double max_lookahead_dist;
    double lookahead_ratio;
    double Kp;
    int window_size;
    double max_steering_angle;
    double max_lateral_acc;  // Computed from friction, gravity and safety factor
    double min_speed;
    double max_speed;
    bool reactive;


    // Topics and Paths
    std::string csv_path;
    std::string odom_topic;
    std::string drive_topic;
    std::string graph_topic;
    std::string goalpoint_topic;
    std::string map_frame;
    std::string car_frame;

    // ROS2 Interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<interfaces_pkg::msg::GoalPoint>::SharedPtr goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graph_pub_;
   
    // Tf2 Listener
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Methods
    double to_radians(double degrees);
    double to_degrees(double radians);
    double p2pdist(double x1, double x2, double y1, double y2);

    void map2car();

    int calculate_n_pathpoints(const std::string& csv_path);
    int load_pathpoints2memory();

    void get_lookahead_point();
    void graph_lookahead_point();
    void steering_angle_calculation();

    void get_closest_pathpoint();
    void graph_closest_pathpoint();
    void speed_calculation();

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);


    // Utility transforms
    Eigen::Matrix3d quaternionToMatrix(const geometry_msgs::msg::Quaternion& q);
    Eigen::Vector3d transform_to_car_frame(const Eigen::Vector3d& point_map);
};
