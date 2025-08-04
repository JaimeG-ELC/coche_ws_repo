#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class TestServoNode : public rclcpp::Node
{
public:
    TestServoNode() : Node("test_servo_node")
    {
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        
        // Set steering angle and speed
        steering_angle_ = -0.76;  // radians
        speed_ = 0.5;             // m/s

        // Timer to publish at 10 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TestServoNode::publish_steering, this)
        );
    }

private:
    void publish_steering()
    {
        auto msg = ackermann_msgs::msg::AckermannDriveStamped();
        msg.drive.steering_angle = steering_angle_;
        msg.drive.speed = speed_;

        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Published steering: %.2f, speed: %.2f", steering_angle_, speed_);
    }

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double steering_angle_;
    double speed_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestServoNode>());
    rclcpp::shutdown();
    return 0;
}