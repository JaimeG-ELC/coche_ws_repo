#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "/root/coche_ws/install/vesc_msgs/include/vesc_msgs/msg/vesc_imu_stamped.hpp"

class ImuTFBroadcaster : public rclcpp::Node
{
public:
  ImuTFBroadcaster();

private:
  void imuCallback(const vesc_msgs::msg::VescImuStamped::SharedPtr msg);

  rclcpp::Subscription<vesc_msgs::msg::VescImuStamped>::SharedPtr imu_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};