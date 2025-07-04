#include "f1tenth_stack/tf_publisher_node.hpp"

TFPublisherNode::TFPublisherNode() : Node("tf_publisher_node") {
    // Declare and get parameters
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("imu_frame", "imu");
    this->declare_parameter("imu_x", 0.1);
    this->declare_parameter("imu_y", 0.02);
    this->declare_parameter("imu_z", 0.1);
    this->declare_parameter("use_ypr", false);
    this->declare_parameter("print_yaw_rates", false);

    odom_frame_ = this->get_parameter("odom_frame").as_string();
    imu_frame_ = this->get_parameter("imu_frame").as_string();
    imu_x_ = this->get_parameter("imu_x").as_double();
    imu_y_ = this->get_parameter("imu_y").as_double();
    imu_z_ = this->get_parameter("imu_z").as_double();
    use_ypr_ = this->get_parameter("use_ypr").as_bool();
    print_yaw_rates_ = this->get_parameter("print_yaw_rates").as_bool(); 

    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to IMU topic
    imu_sub_ = this->create_subscription<interfaces_pkg::msg::VescImuStamped>(
        "imu", 10,
        std::bind(&TFPublisherNode::imuCallback, this, std::placeholders::_1)
    );
    // imu_std_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    //     "imu", 10,
    //     std::bind(&TFPublisherNode::imuCallback, this, std::placeholders::_1)
    // );

    // Create timer for periodic publishing
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&TFPublisherNode::timerCallback, this));

    // Initialize orientation to identity
    latest_orientation_.x = 0.0;
    latest_orientation_.y = 0.0;
    latest_orientation_.z = 0.0;
    latest_orientation_.w = 1.0;

    last_yaw_rate_ = 0.0;
    last_yaw_rate_stamp_ = this->now();

    RCLCPP_INFO(this->get_logger(), "TF Publisher Node (odom->imu, dynamic) initialized");
}

void TFPublisherNode::imuCallback(const interfaces_pkg::msg::VescImuStamped::SharedPtr msg) {
    latest_stamp_ = msg->header.stamp;

    yaw_rate_ = msg->imu.angular_velocity.z; // Yaw rate

    // Compute yaw acceleration
    yaw_acc_ = 0.0;
    if (last_yaw_rate_stamp_.nanoseconds() > 0) {
        double dt = (msg->header.stamp - last_yaw_rate_stamp_).seconds();
        if (dt > 1e-6) {
            yaw_acc_ = (yaw_rate_ - last_yaw_rate_) / dt;
        }
    }

    last_yaw_rate_ = yaw_rate_;
    last_yaw_rate_stamp_ = msg->header.stamp;

    if (use_ypr_) {
        // Convert YPR to quaternion
        double yaw = msg->imu.ypr.z;
        double pitch = msg->imu.ypr.y;
        double roll = msg->imu.ypr.x;
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        latest_orientation_.x = q.x();
        latest_orientation_.y = q.y();
        latest_orientation_.z = q.z();
        latest_orientation_.w = q.w();
    } else {
        // Use quaternion directly
        latest_orientation_ = msg->imu.orientation;
    }
}

void TFPublisherNode::timerCallback() {
    geometry_msgs::msg::TransformStamped transform;
    if (latest_stamp_.nanoseconds() > 0) {
        transform.header.stamp = latest_stamp_;
    } else {
        transform.header.stamp = this->get_clock()->now();
    }
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = imu_frame_;

    // Set translation (static offset)
    transform.transform.translation.x = imu_x_;
    transform.transform.translation.y = imu_y_;
    transform.transform.translation.z = imu_z_;

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

    // Create quaternion with only yaw
    tf2::Quaternion q_yaw;
    q_yaw.setRPY(0.0, 0.0, yaw);

    transform.transform.rotation.x = q_yaw.x();
    transform.transform.rotation.y = q_yaw.y();
    transform.transform.rotation.z = q_yaw.z();
    transform.transform.rotation.w = q_yaw.w();

    tf_broadcaster_->sendTransform(transform);

    RCLCPP_INFO(
        this->get_logger(),
        "Published YAW-only transform from '%s' to '%s' \n (yaw: %.2f deg)",
        odom_frame_.c_str(), imu_frame_.c_str(), yaw_deg
    );
        
    if (print_yaw_rates_) {
        double yaw_rate_deg = yaw_rate_ * 180.0 / M_PI;
        double yaw_acc_deg = yaw_acc_ * 180.0 / M_PI;
        RCLCPP_INFO(this->get_logger(),
            "(yaw rate: %.4f deg/s)\n (yaw acceleration: %.4f deg/s^2)",
            yaw_rate_deg, yaw_acc_deg);
    }
}
