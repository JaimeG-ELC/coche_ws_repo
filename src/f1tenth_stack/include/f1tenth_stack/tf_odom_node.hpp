#ifndef TF_ODOM_NODE_HPP_
#define TF_ODOM_NODE_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <interfaces_pkg/msg/vesc_imu_stamped.hpp>  

#include <geometry_msgs/msg/vector3.hpp>

class TFOdomNode : public rclcpp::Node {
public:
    TFOdomNode();

private:
    // Parameters for odom->imu transform
    std::string odom_frame_;
    std::string base_link_frame_;

    double x_, y_, yaw_;
    bool has_prev_time_;
    rclcpp::Time prev_time_;

    // Timer

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_sub_;

    // Callbacks
    void vescCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg);
};

#endif // TF_ODOM_NODE_HPP_