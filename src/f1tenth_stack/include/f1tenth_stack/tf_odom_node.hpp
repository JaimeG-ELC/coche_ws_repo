#ifndef TF_ODOM_NODE_HPP_
#define TF_ODOM_NODE_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

class TFOdomNode : public rclcpp::Node {
public:
    TFOdomNode();

private:
    // Parameters for odom->imu transform

    std::string odom_topic_, odom_output_topic_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::TransformStamped static_transform_stamped_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom0_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom0_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom1_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom1_pub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;


    // Callbacks
    void odom0Callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void odom1Callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

};

#endif // TF_ODOM_NODE_HPP_