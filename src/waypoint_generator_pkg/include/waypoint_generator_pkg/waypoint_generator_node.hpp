#include <fstream> // Required to work with csv files
#include <iostream>
#include <string>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class WayPointGenerator : public rclcpp::Node
{
    public:
        WayPointGenerator();

    private:
        // Required Variables
        std::string csv_path;
        double min_distance;
        double prev_x;
        double prev_y;
        std::ofstream csv_odom;
        std::string map_frame;    // New parameter
        std::string car_frame;    // New parameter

        // TF2 members
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;

        // Timer callback
        void timer_callback();
};
