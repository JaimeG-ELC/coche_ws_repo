#include "manual_control_pkg/manual_control_node.hpp"

ManualControlNode::ManualControlNode() : Node("manual_control_node"){
    // Declare and retrieve parameters
    this->declare_parameter<int>("lb_button_idx", 4);
    this->declare_parameter<int>("rb_button_idx", 5);
    // this->declare_parameter<int>("brake_button_idx", 3);
    this->declare_parameter<int>("rt_axis_idx", 5);
    this->declare_parameter<int>("lt_axis_idx", 2);
    this->declare_parameter<int>("left_horizontal_axis_idx", 0);
    this->declare_parameter<std::string>("joy_topic", "/joy");
    this->declare_parameter<std::string>("drive_topic", "/drive");
    this->declare_parameter<std::string>("ackermann_cmd_topic", "/ackermann_cmd");
    this->declare_parameter<double>("throttle_gain", 1.0);
    this->declare_parameter<double>("throttle_multiplier", 1.0);
    this->declare_parameter<double>("steering_gain", 0.2567);
    this->declare_parameter<double>("steering_offset", 0.0);
    this->declare_parameter<double>("constant_throttle", 1.0);

    // Get parameters
    lb_button_idx_ = this->get_parameter("lb_button_idx").as_int();
    rb_button_idx_ = this->get_parameter("rb_button_idx").as_int();
    // brake_button_idx_ = this->get_parameter("brake_button_idx").as_int();
    rt_axis_idx_ = this->get_parameter("rt_axis_idx").as_int();
    lt_axis_idx_ = this->get_parameter("lt_axis_idx").as_int();
    left_horizontal_axis_idx_ = this->get_parameter("left_horizontal_axis_idx").as_int();
    joy_topic_ = this->get_parameter("joy_topic").as_string();
    drive_topic_ = this->get_parameter("drive_topic").as_string();
    ackermann_cmd_topic_ = this->get_parameter("ackermann_cmd_topic").as_string();
    throttle_gain_ = this->get_parameter("throttle_gain").as_double();
    throttle_multiplier_ = this->get_parameter("throttle_multiplier").as_double();
    steering_gain_ = this->get_parameter("steering_gain").as_double();
    steering_offset_ = this->get_parameter("steering_offset").as_double();
    constant_throttle_ = this->get_parameter("constant_throttle").as_double();

    // Initialize publishers and subscribers
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_, 10, std::bind(&ManualControlNode::joyCallback, this, std::placeholders::_1));
    drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic_, 10, std::bind(&ManualControlNode::driveCallback, this, std::placeholders::_1));

    ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(ackermann_cmd_topic_, 10);
    enable_button_pub_ = this->create_publisher<std_msgs::msg::Int8>("/enable_0", 10);
    enable_button1_pub_ = this->create_publisher<std_msgs::msg::Int8>("/enable_1", 10);

    button_pressed_ = false;
    prev_throttle_gain_button_value_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "constant_throttle_: %f", constant_throttle_);
    RCLCPP_INFO(get_logger(), "Manual control node initialized");
}

float ManualControlNode::linear_map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void ManualControlNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy) {

    publishEnableButtons(joy);

    button_pressed_ = joy->buttons[lb_button_idx_];

    ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;
    ackermann_msg.header.stamp = now();
    ackermann_msg.header.frame_id = "base_link";

    ackermann_msg.drive.speed = calculateThrottle(joy);
    ackermann_msg.drive.steering_angle = -joy->axes[left_horizontal_axis_idx_] * steering_gain_ + steering_offset_;

    // if (joy->buttons[brake_button_idx_]) {
    //     ackermann_msg.drive.acceleration = 2.0;  // Stop the vehicle if brake button is pressed
    //     RCLCPP_INFO(get_logger(), "BRAKE!!");
    // }

    // RCLCPP_DEBUG(get_logger(), "Speed: %f, Steering Angle: %f, Brake: %f", ackermann_msg.drive.speed, ackermann_msg.drive.steering_angle, ackermann_msg.drive.acceleration);
    RCLCPP_DEBUG(get_logger(), "Speed: %f, Steering Angle: %f", ackermann_msg.drive.speed, ackermann_msg.drive.steering_angle);
    ackermann_pub_->publish(ackermann_msg);
    handleDriveMultiplierAdjustment(joy);
}

void ManualControlNode::publishEnableButtons(const sensor_msgs::msg::Joy::SharedPtr& joy) {
    std_msgs::msg::Int8 enable_msg;
    
    enable_msg.data = joy->buttons[0];
    enable_button_pub_->publish(enable_msg);

    enable_msg.data = joy->buttons[1];
    enable_button1_pub_->publish(enable_msg);
}

double ManualControlNode::calculateThrottle(const sensor_msgs::msg::Joy::SharedPtr& joy) {
    const bool both_buttons_pressed = joy->buttons[lb_button_idx_] && joy->buttons[rb_button_idx_];
    const float multiplier = both_buttons_pressed ? throttle_multiplier_ : 1.0;

     if (joy->buttons[rb_button_idx_]) 
    {
        return constant_throttle_;  // Override speed if RB is pressed

    }   else if (joy->axes[lt_axis_idx_] != 1.0) 
    {
        return -linear_map(joy->axes[lt_axis_idx_], 1, -1, 0, 1) * throttle_gain_ * multiplier;
    }

    return linear_map(joy->axes[rt_axis_idx_], 1, -1, 0, 1) * throttle_gain_ * multiplier;
}

void ManualControlNode::handleDriveMultiplierAdjustment(const sensor_msgs::msg::Joy::SharedPtr& joy) {
    constexpr int dpad_vertical_axis = 7;
    float current_value = joy->axes[dpad_vertical_axis];

    if (current_value == 1.0 && prev_throttle_gain_button_value_ == 0.0) {
        throttle_gain_ += 0.05;
        RCLCPP_INFO(get_logger(), "Multiplier increased to %f", throttle_gain_);
    } else if (current_value == -1.0 && prev_throttle_gain_button_value_ == 0.0) {
        throttle_gain_ -= 0.05;
        RCLCPP_INFO(get_logger(), "Multiplier decreased to %f", throttle_gain_);
    }

    prev_throttle_gain_button_value_ = current_value;
}

void ManualControlNode::driveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr drive) {
    if (!button_pressed_) return;

    auto modified_drive = *drive;
    ackermann_pub_->publish(modified_drive);
}