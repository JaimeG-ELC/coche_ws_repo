#include "f1tenth_stack/imu_tf_broadcaster.hpp"

ImuTFBroadcaster::ImuTFBroadcaster()
: Node("imu_tf_broadcaster")
{
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  imu_sub_ = this->create_subscription<vesc_msgs::msg::VescImuStamped>(
    "/sensors/imu/raw", 10,
    std::bind(&ImuTFBroadcaster::imuCallback, this, std::placeholders::_1)
  );
}

void ImuTFBroadcaster::imuCallback(const vesc_msgs::msg::VescImuStamped::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped transform;

  transform.header.stamp = msg->header.stamp;
  transform.header.frame_id = "map";       // Reference frame
  transform.child_frame_id = "imu";        // IMU frame

  // Position: assume fixed if not provided
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;

  // Use orientation from IMU message
  transform.transform.rotation = msg->imu.orientation;

  tf_broadcaster_->sendTransform(transform);
}


// Main
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
