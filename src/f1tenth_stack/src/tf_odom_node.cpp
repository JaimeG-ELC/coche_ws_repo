#include "f1tenth_stack/tf_odom_node.hpp"

TFOdomNode::TFOdomNode()
    : Node("tf_odom_node"),
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
        std::bind(&TFOdomNode::vescCallback, this, std::placeholders::_1));
        
    // servo_sub_ = this->create_subscription<Float64>(
    //   "sensors/servo_position_command", 10, std::bind(&TFOdomNode::servoCmdCallback, this, std::placeholders::_1));

    x_ = y_ = yaw = 0.0;
    has_prev_time_ = false;

    RCLCPP_INFO(this->get_logger(), "TFOdomNode node initialized");
}

void TFOdomNode::vescCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
{
    rclcpp::Time current_time = msg->header.stamp;

    if(!has_prev_time_) {
        prev_time_ = current_time;
        has_prev_time_ = true;
        return;
    }
    // Convert ERPM to m/s
    double speed = -(msg->state.speed + speed_to_erpm_offset_) / speed_to_erpm_gain_;
    if (std::fabs(speed) < 0.05) {
        speed = 0.0;
    }

    double dt = (current_time - prev_time_).seconds();
    x_ += speed * cos(yaw_) * dt;
    y_ += speed * sin(yaw_) * dt;


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

    RCLCPP_INFO(
        this->get_logger(),
        "Published transform from '%s' to '%s' at time %.2f: (x: %.2f, y: %.2f, yaw: %.2f)",
        odom_frame_.c_str(), base_frame_.c_str(),
        current_time.seconds(), x_, y_, yaw_
    );
}

// The main function
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}