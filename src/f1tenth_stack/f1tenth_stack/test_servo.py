import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class TestServoNode(Node):
    def __init__(self):
        super().__init__('test_servo_node')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(0.1, self.publish_steering)
        self.steering_angle = -0.76  # radians
        # self.steering_angle = 0.76  # radians
        self.speed = 0.5           # m/s, adjust as needed

    def publish_steering(self):
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = self.steering_angle
        msg.drive.speed = self.speed
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published steering: {self.steering_angle}, speed: {self.speed}')

def main(args=None):
    rclpy.init(args=args)
    node = TestServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()