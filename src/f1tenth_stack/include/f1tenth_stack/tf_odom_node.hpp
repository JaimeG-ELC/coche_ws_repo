#ifndef TF_ODOM_NODE_HPP_
#define TF_ODOM_NODE_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

class TFOdomNode : public rclcpp::Node {
public:
    TFOdomNode();

private:
    // Parameters for odom->imu transform
    std::string odom_frame_;
    std::string base_frame_;

    double speed_to_erpm_gain_, speed_to_erpm_offset_;
    double steering_to_servo_gain_, steering_to_servo_offset_;
    double wheelbase_;


    double x_, y_, yaw_;    
    bool has_prev_time_;
    rclcpp::Time prev_time_;

    // Timer

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_sub_;


    // Callbacks
    void servoCmdCallback(const std_msgs::msg::Float64::SharedPtr servo);
    void vescCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg);

};

#endif // TF_ODOM_NODE_HPP_