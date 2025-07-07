#include "f1tenth_stack/tf_publisher_node.hpp"

TFPublisherNode::TFPublisherNode() : Node("tf_publisher_node") {
    // Declare and get parameters
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("imu_frame", "imu");

    odom_frame_ = this->get_parameter("odom_frame").as_string();
    imu_frame_ = this->get_parameter("imu_frame").as_string();

    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to IMU topic
    // imu_sub_ = this->create_subscription<interfaces_pkg::msg::VescImuStamped>(
    //     "imu", 10,
    //     std::bind(&TFPublisherNode::imuCallback, this, std::placeholders::_1)
    // );
    imu_std_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "sensors/imu/raw", 10,
        std::bind(&TFPublisherNode::imuCallback, this, std::placeholders::_1)
    );

    // Create timer for periodic publishing
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&TFPublisherNode::timerCallback, this));

    // Initialize orientation to identity
    latest_orientation_.x = 0.0;
    latest_orientation_.y = 0.0;
    latest_orientation_.z = 0.0;
    latest_orientation_.w = 1.0;

    velocity_.x = velocity_.y  = 0.0;
    position_.x = position_.y = 0.0;
    has_prev_time_ = false;

    RCLCPP_INFO(this->get_logger(), "TF Publisher Node (odom->imu, dynamic) initialized");
}

void TFPublisherNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    latest_stamp_ = msg->header.stamp;
    latest_orientation_ = msg->orientation;
    latest_linear_accel_ = msg->linear_acceleration;
}

void TFPublisherNode::timerCallback() {
    geometry_msgs::msg::TransformStamped transform;

    // Use latest IMU timestamp if available, otherwise current clock time
    rclcpp::Time current_time = latest_stamp_.nanoseconds() > 0
        ? latest_stamp_
        : this->get_clock()->now();

    transform.header.stamp = current_time;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = imu_frame_;

    // Integrate motion
    if (has_prev_time_) {
        double dt = (current_time - prev_time_).seconds();

        // Acceleration integration (simplistic)
        velocity_.x += latest_linear_accel_.x * dt;
        velocity_.y += latest_linear_accel_.y * dt;

        position_.x += velocity_.x * dt;
        position_.y += velocity_.y * dt;
    } else {
        has_prev_time_ = true;
    }
    prev_time_ = current_time;

    // Set dynamic translation from integration
    transform.transform.translation.x = position_.x;
    transform.transform.translation.y = position_.y;
    transform.transform.translation.z = position_.z;

    // Extract yaw from IMU orientation
    tf2::Quaternion q_imu(
        latest_orientation_.x,
        latest_orientation_.y,
        latest_orientation_.z,
        latest_orientation_.w
    );

    tf2::Quaternion q_flip;
    q_flip.setRPY(0, 0, M_PI);  // 180 deg Z

    tf2::Quaternion q_final = q_flip * q_imu;
    q_final.normalize();

    transform.transform.rotation.x = q_final.x();
    transform.transform.rotation.y = q_final.y();
    transform.transform.rotation.z = q_final.z();
    transform.transform.rotation.w = q_final.w();

    tf_broadcaster_->sendTransform(transform);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q_final).getRPY(roll, pitch, yaw);

    // Convert yaw to degrees for printing
    double yaw_deg = yaw * 180.0 / M_PI;

    RCLCPP_INFO(
        this->get_logger(),
        "Published YAW-only transform from '%s' to '%s' (yaw: %.2f deg)",
        odom_frame_.c_str(), imu_frame_.c_str(), yaw_deg
    );

    RCLCPP_INFO(
        this->get_logger(),
        "TF (odom -> imu): x=%.2f y=%.2f z=%.2f",
        position_.x, position_.y, position_.z
    );
}