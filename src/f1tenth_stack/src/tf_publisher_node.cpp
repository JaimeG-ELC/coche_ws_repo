#include "map_to_odom_tf_broadcaster/broadcaster_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>

MapToOdomBroadcaster::MapToOdomBroadcaster()
: Node("map_to_odom_tf_broadcaster")
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&MapToOdomBroadcaster::broadcastTransform, this)
  );
}

void MapToOdomBroadcaster::broadcastTransform()
{
  geometry_msgs::msg::TransformStamped map_to_base, odom_to_base;

  try {
    map_to_base = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    odom_to_base = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
    return;
  }

  // Convert to tf2
  tf2::Transform tf_map_base, tf_odom_base;
  tf2::fromMsg(map_to_base.transform, tf_map_base);
  tf2::fromMsg(odom_to_base.transform, tf_odom_base);

  tf2::Transform tf_base_odom = tf_odom_base.inverse();
  tf2::Transform tf_map_odom = tf_map_base * tf_base_odom;

  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.stamp = this->get_clock()->now();
  map_to_odom.header.frame_id = "map";
  map_to_odom.child_frame_id = "odom";
  map_to_odom.transform = tf2::toMsg(tf_map_odom);

  tf_broadcaster_->sendTransform(map_to_odom);
}
