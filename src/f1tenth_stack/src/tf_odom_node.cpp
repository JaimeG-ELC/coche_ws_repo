#include "f1tenth_stack/tf_odom_node.hpp"

TFOdomNode::TFOdomNode()
    : Node("tf_odom_node")
{
    // Declare and get parameters
    this->declare_parameter("odom_topic", "/pf/pose/odom");
    this->declare_parameter("odom_output_topic", "/pf/tf_odom");

    odom_topic_ = this->get_parameter("odom_topic").as_string();
    odom_output_topic_ = this->get_parameter("odom_output_topic").as_string(); 

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    odom0_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&TFOdomNode::odom0Callback, this, std::placeholders::_1));
        
    odom0_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        odom_output_topic_, 10);

    odom1_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&TFOdomNode::odom1Callback, this, std::placeholders::_1));
    
    odom1_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/tf_odom", 10);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&TFOdomNode::scanCallback, this, std::placeholders::_1));
    
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/tf_scan", 10);


    RCLCPP_INFO(this->get_logger(), "TFOdomNode node initialized");
}

void TFOdomNode::odom0Callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{

    try
    {  // 1. Get laser pose in map frame
        tf2::Transform tf_map_to_laser;
        tf2::fromMsg(msg->pose.pose, tf_map_to_laser);

        // 2. Define fixed transform from laser to virtual frame
        tf2::Transform tf_laser_to_tf_odom;
        tf_laser_to_tf_odom.setOrigin(tf2::Vector3(-0.36, 0.0, -0.12));
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);  // no rotation
        tf_laser_to_tf_odom.setRotation(q);

        // 3. Compose the transform: map → tf_odom = map → laser * laser → tf_odom
        tf2::Transform tf_map_to_tf_odom = tf_map_to_laser * tf_laser_to_tf_odom;

        // 4. Convert back to pose
        geometry_msgs::msg::Pose tf_odom_pose;
        tf_odom_pose.position.x = tf_map_to_tf_odom.getOrigin().x();
        tf_odom_pose.position.y = tf_map_to_tf_odom.getOrigin().y();
        tf_odom_pose.position.z = tf_map_to_tf_odom.getOrigin().z();
        tf_odom_pose.orientation.x = tf_map_to_tf_odom.getRotation().x();
        tf_odom_pose.orientation.y = tf_map_to_tf_odom.getRotation().y();
        tf_odom_pose.orientation.z = tf_map_to_tf_odom.getRotation().z();
        tf_odom_pose.orientation.w = tf_map_to_tf_odom.getRotation().w();

        // 5. Publish new odometry
        nav_msgs::msg::Odometry odom0_msg = *msg;
        odom0_msg.pose.pose = tf_odom_pose;
        odom0_msg.header.frame_id = "odom";  // or whatever you want to call it
        odom0_msg.child_frame_id = "base_link";  // or whatever you want to call it
        odom0_pub_->publish(odom0_msg);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
    }
}

void TFOdomNode::odom1Callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    nav_msgs::msg::Odometry odom1_msg = *msg;
    odom1_msg.header.frame_id = "map";
    odom1_pub_->publish(odom1_msg);
}

void TFOdomNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    // Transform scan using a static transform from laser to base_link
    // We'll update the header.frame_id and header.stamp, but the scan points themselves are not transformed here.
    // For a true point transformation, you'd need to transform each range to base_link coordinates.

    sensor_msgs::msg::LaserScan scan_msg = *msg;
    scan_msg.header.frame_id = "laser"; // Set to target frame

    // Optionally, update the timestamp if needed
    scan_msg.header.stamp = this->now();

    scan_pub_->publish(scan_msg);
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