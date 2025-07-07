#include "f1tenth_stack/base_to_odom_tf_publisher_node.hpp"

BaseToOdomTFPublisher::BaseToOdomTFPublisher()
    : Node("base_to_odom_tf_publisher"),
      x_(0.0), y_(0.0), yaw_(0.0), has_prev_time_(false)
{
    // Declare and get parameters
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("speed_to_erpm_gain", 7528.0);
    this->declare_parameter("speed_to_erpm_offset", 0.0);

    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    speed_to_erpm_gain_ = this->get_parameter("speed_to_erpm_gain").as_double();
    speed_to_erpm_offset_ = this->get_parameter("speed_to_erpm_offset").as_double();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    vesc_sub_ = this->create_subscription<vesc_msgs::msg::VescStateStamped>(
        "sensors/core", 10,
        std::bind(&BaseToOdomTFPublisher::vescCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "BaseToOdomTFPublisher node initialized");
}

void BaseToOdomTFPublisher::vescCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
{
    rclcpp::Time current_time = msg->header.stamp;

    // Convert ERPM to m/s
    double speed = -(msg->state.speed + speed_to_erpm_offset_) / speed_to_erpm_gain_;
    if (std::fabs(speed) < 0.05) {
        speed = 0.0;
    }

    if (has_prev_time_) {
        double dt = (current_time - prev_time_).seconds();
        x_ += speed * cos(yaw_) * dt;
        y_ += speed * sin(yaw_) * dt;
        // No yaw change assumed here
    } else {
        has_prev_time_ = true;
    }

    prev_time_ = current_time;

    // Prepare transform message
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
}
