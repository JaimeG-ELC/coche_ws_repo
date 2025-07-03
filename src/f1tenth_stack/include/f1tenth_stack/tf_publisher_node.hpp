#ifndef TF_PUBLISHER_NODE_HPP_
#define TF_PUBLISHER_NODE_HPP_

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class MapToOdomBroadcaster : public rclcpp::Node
{
public:
  MapToOdomBroadcaster();

private:
  void broadcastTransform();

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};


#endif // TF_PUBLISHER_NODE_HPP_ 