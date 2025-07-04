#ifndef TF_PUBLISHER_NODE_HPP_
#define TF_PUBLISHER_NODE_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <interfaces_pkg/msg/vesc_imu_stamped.hpp>  

class TFPublisherNode : public rclcpp::Node {
public:
    TFPublisherNode();

private:
    // Parameters for odom->imu transform
    std::string odom_frame_;
    std::string imu_frame_;
    double imu_x_;
    double imu_y_;
    double imu_z_;

    // Latest IMU orientation
    geometry_msgs::msg::Quaternion latest_orientation_;
    rclcpp::Time latest_stamp_;

    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // IMU subscriber
    rclcpp::Subscription<interfaces_pkg::msg::VescImuStamped>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_std_sub_;

    // Callbacks
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void timerCallback();

    bool use_ypr_;
    bool print_yaw_rates_;
};

#endif // TF_PUBLISHER_NODE_HPP_