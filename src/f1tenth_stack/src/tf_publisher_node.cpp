#include "f1tenth_stack/tf_publisher_node.hpp"

TFPublisherNode::TFPublisherNode() : Node("tf_publisher_node") {
    // Declare and get parameters
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("imu_frame", "imu");
    this->declare_parameter("imu_x", 0.0);
    this->declare_parameter("imu_y", 0.0);
    this->declare_parameter("imu_z", 0.0);

    odom_frame_ = this->get_parameter("odom_frame").as_string();
    imu_frame_ = this->get_parameter("imu_frame").as_string();
    imu_x_ = this->get_parameter("imu_x").as_double();
    imu_y_ = this->get_parameter("imu_y").as_double();
    imu_z_ = this->get_parameter("imu_z").as_double();


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

    latest_linear_acceleration_.x = latest_linear_acceleration_.y = 0.0;
    latest_linear_velocity_.x = latest_linear_velocity_.y = 0.0;
    latest_position_.x = latest_position_.y = 0.0;


    RCLCPP_INFO(this->get_logger(), "TF Publisher Node (odom->imu, dynamic) initialized");
}

void TFPublisherNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    latest_stamp_ = msg->header.stamp;
    latest_orientation_ = msg->orientation;
    latest_linear_acceleration_ = msg->linear_acceleration;

    static bool first_imu_received = false;
    if (!first_imu_received) {
        prev_time_ = latest_stamp_;
        first_imu_received = true;
    }
}

void TFPublisherNode::timerCallback() {
    geometry_msgs::msg::TransformStamped transform;

    if (latest_stamp_.nanoseconds() > 0) {
        transform.header.stamp = latest_stamp_;
    } else {
        return;
    }
        
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = imu_frame_;

    double dt = (latest_stamp_ - prev_time_).seconds();
    prev_time_ = latest_stamp_;

    // 1. Convert IMU orientation to rotation matrix
    tf2::Quaternion q_rot(
        latest_orientation_.x,
        latest_orientation_.y,
        latest_orientation_.z,
        latest_orientation_.w
    );
    tf2::Matrix3x3 rotation(q_rot);

    // 2. Rotate acceleration to world frame
    tf2::Vector3 acc_body(
        latest_linear_acceleration_.x,
        latest_linear_acceleration_.y,
        latest_linear_acceleration_.z
    );
    // tf2::Vector3 acc_world = rotation * acc_body;
    tf2::Vector3 acc_world = acc_body;

    // 3. Optional: Remove gravity if z is affected
    acc_world.setZ(0.0); // assuming you only want to integrate XY

    double vel_x, vel_y;

    if (std::abs(acc_world.x()) < 0.05){
        vel_x = 0.0;
    } else{
        vel_x = -acc_world.x() * 100;
    }

    if (std::abs(acc_world.y()) < 0){
        vel_y = 0.0;
    } else{
        vel_y = acc_world.y() * 100;
    }

    latest_linear_velocity_.x += vel_x * dt;
    latest_linear_velocity_.y += vel_y * dt;

    latest_position_.x += latest_linear_velocity_.x * dt;
    latest_position_.y += latest_linear_velocity_.y * dt;

    RCLCPP_INFO(
        this->get_logger(),
        "Acceleration: (%.2f, %.2f, %.2f)",
        acc_world.x(), acc_world.y(), acc_world.z()
    );
    RCLCPP_INFO(
        this->get_logger(),
        "Acc_mult: (%.2f, %.2f, %.2f)",
        vel_x, vel_y, 0.0
    );
    RCLCPP_INFO(
        this->get_logger(),
        "Velocity: (%.2f, %.2f, %.2f)",
        latest_linear_velocity_.x, latest_linear_velocity_.y, 0.0
    );

    // Set dynamic position from IMU integration
    transform.transform.translation.x = latest_position_.x;
    transform.transform.translation.y = latest_position_.y;
    transform.transform.translation.z = imu_z_;  // keep static Z offset

    // Extract yaw from IMU orientation
    tf2::Quaternion q_imu(
        latest_orientation_.x,
        latest_orientation_.y,
        latest_orientation_.z,
        latest_orientation_.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_imu).getRPY(roll, pitch, yaw);

    // Convert yaw to degrees for printing
    double yaw_deg = yaw * 180.0 / M_PI;
    tf2::Quaternion q_yaw;
    q_yaw.setRPY(roll, pitch, yaw);

    // Apply an additional 180-degree rotation around X
    tf2::Quaternion q_flip;
    q_flip.setRPY(0, 0, M_PI);  // 180 deg around X

    // Final rotation: flip * yaw
    tf2::Quaternion q_final = q_flip * q_yaw;
    q_final.normalize();

    // Set the transform rotation
    transform.transform.rotation.x = q_final.x();
    transform.transform.rotation.y = q_final.y();
    transform.transform.rotation.z = q_final.z();
    transform.transform.rotation.w = q_final.w();

    tf_broadcaster_->sendTransform(transform);

    RCLCPP_INFO(
        this->get_logger(),
        "Published YAW-only transform from '%s' to '%s' \n (yaw: %.2f deg)",
        odom_frame_.c_str(), imu_frame_.c_str(), yaw_deg
    );       
    RCLCPP_INFO(
        this->get_logger(),
        "Position: (%.2f, %.2f, %.2f)",
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z
    );
}