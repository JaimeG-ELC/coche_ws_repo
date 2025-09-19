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

class TFImuNode : public rclcpp::Node {
public:
    TFImuNode();

private:
    // Parameters for odom->imu transform
    std::string imu_frame_;
    std::string imu_out_frame_;

    double imu_x_, imu_y_, imu_z_;
    double roll_offset_, pitch_offset_, yaw_offset_;

    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // IMU subscriber & publisher
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_std_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // Callbacks
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
};

#endif // TF_PUBLISHER_NODE_HPP_