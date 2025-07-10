#include "f1tenth_stack/tf_imu_node.hpp"

TFImuNode::TFImuNode() : Node("tf_imu_node") {
    // Declare and get parameters
    this->declare_parameter("imu_frame", "sensors/imu/raw");
    this->declare_parameter("imu_out_frame", "sensors/imu/transformed");
    this->declare_parameter("imu_x", 0.0);
    this->declare_parameter("imu_y", 0.0);
    this->declare_parameter("imu_z", 0.0);
    this->declare_parameter("roll_offset", 0.0);
    this->declare_parameter("pitch_offset", 0.0);
    this->declare_parameter("yaw_offset", 0.0);

    imu_frame_ = this->get_parameter("imu_frame").as_string();
    imu_out_frame_ = this->get_parameter("imu_out_frame").as_string();
    imu_x_ = this->get_parameter("imu_x").as_double();
    imu_y_ = this->get_parameter("imu_y").as_double();
    imu_z_ = this->get_parameter("imu_z").as_double();
    roll_offset_ = this->get_parameter("roll_offset").as_double();
    pitch_offset_ = this->get_parameter("pitch_offset").as_double();
    yaw_offset_ = this->get_parameter("yaw_offset").as_double();

    RCLCPP_INFO(this->get_logger(), "TF IMU Node parameters:");
    RCLCPP_INFO(this->get_logger(), "  imu_frame: %s", imu_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  imu_out_frame: %s", imu_out_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  imu_x: %.2f", imu_x_);
    RCLCPP_INFO(this->get_logger(), "  imu_y: %.2f", imu_y_);
    RCLCPP_INFO(this->get_logger(), "  imu_z: %.2f", imu_z_);
    RCLCPP_INFO(this->get_logger(), "  roll_offset: %.2f", roll_offset_);
    RCLCPP_INFO(this->get_logger(), "  pitch_offset: %.2f", pitch_offset_);
    RCLCPP_INFO(this->get_logger(), "  yaw_offset: %.2f", yaw_offset_);

    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to IMU topic
    // imu_sub_ = this->create_subscription<interfaces_pkg::msg::VescImuStamped>(
    //     "imu", 10,
    //     std::bind(&TFImuNode::imuCallback, this, std::placeholders::_1)
    // );
    imu_std_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_frame_, 10,
        std::bind(&TFImuNode::imuCallback, this, std::placeholders::_1)
    );
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_out_frame_, 10);


    RCLCPP_INFO(this->get_logger(), "TF Imu Node (imu/raw->imu/transformed) initialized");
}

void TFImuNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = msg->header.stamp;
    transform.header.frame_id = imu_frame_;
    transform.child_frame_id = imu_out_frame_;

    transform.transform.translation.x = imu_x_;
    transform.transform.translation.y = imu_y_;
    transform.transform.translation.z = imu_z_;

    tf2::Quaternion q_orig, q_offset, q_new;
    q_orig.setX(msg->orientation.x);
    q_orig.setY(msg->orientation.y);
    q_orig.setZ(msg->orientation.z);
    q_orig.setW(msg->orientation.w);

    q_offset.setRPY(roll_offset_, pitch_offset_, yaw_offset_);
    q_new = q_offset * q_orig;
    q_new.normalize();

    transform.transform.rotation.x = q_new.x();
    transform.transform.rotation.y = q_new.y();
    transform.transform.rotation.z = q_new.z();
    transform.transform.rotation.w = q_new.w();

    tf_broadcaster_->sendTransform(transform);

    // Republish transformed IMU
    auto imu_out = *msg;
    imu_out.header.frame_id = imu_out_frame_;
    imu_out.orientation.x = q_new.x();
    imu_out.orientation.y = q_new.y();
    imu_out.orientation.z = q_new.z();
    imu_out.orientation.w = q_new.w();

    imu_pub_->publish(imu_out);

}

// The main function
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFImuNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}