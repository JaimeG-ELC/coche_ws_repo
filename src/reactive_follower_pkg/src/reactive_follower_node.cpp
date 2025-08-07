#include "reactive_follower_pkg/reactive_follower_node.hpp"

ReactiveFollowerNode::ReactiveFollowerNode() : Node("reactive_follower") {
    latest_scan_msg_ = nullptr;
    // Declare and retrieve parameters
    this->declare_parameter("lidarscan_topic", "/scan");
    this->declare_parameter("drive_topic", "/drive");
    this->declare_parameter("goalpoint_topic", "/goalpoint");
    this->declare_parameter("laser_frame", "laser");
    this->declare_parameter("car_frame", "base_link");

    this->declare_parameter("lidar_angle", 270.0);
    this->declare_parameter("lidar_angle_front_car", 135.0);
    this->declare_parameter("lidar_scans", 1180);

    this->declare_parameter("max_speed", 2.0);
    this->declare_parameter("min_speed", 0.5);
    this->declare_parameter("max_steering_angle", 14.705);
    this->declare_parameter("bubble_radius", 50);
    this->declare_parameter("processed_angle", 135.0);
    this->declare_parameter("safety_distance_min", 0.6);
    this->declare_parameter("safety_distance_threshold", 2.0);
    this->declare_parameter("safety_distance_gain", 0.2);
    this->declare_parameter("vehicle_width", 0.3);

    this->declare_parameter("max_lidar_distance", 20.0);
    this->declare_parameter("weight_speed", 0.5);
    this->declare_parameter("weight_steering", 0.5);

    lidarscan_topic = this->get_parameter("lidarscan_topic").as_string();
    drive_topic = this->get_parameter("drive_topic").as_string();
    goalpoint_topic = this->get_parameter("goalpoint_topic").as_string();
    laser_frame = this->get_parameter("laser_frame").as_string();
    car_frame = this->get_parameter("car_frame").as_string();

    lidar_angle = this->get_parameter("lidar_angle").as_double();
    lidar_angle_front_car = this->get_parameter("lidar_angle_front_car").as_double();
    lidar_scans = this->get_parameter("lidar_scans").as_int(); // <-- Change to as_int()

    max_speed = this->get_parameter("max_speed").as_double();
    min_speed = this->get_parameter("min_speed").as_double();
    max_steering_angle = this->get_parameter("max_steering_angle").as_double();
    bubble_radius = this->get_parameter("bubble_radius").as_int();
    processed_angle = this->get_parameter("processed_angle").as_double();
    safety_distance_min = this->get_parameter("safety_distance_min").as_double();
    safety_distance_threshold = this->get_parameter("safety_distance_threshold").as_double();
    safety_distance_gain = this->get_parameter("safety_distance_gain").as_double();
    vehicle_width = this->get_parameter("vehicle_width").as_double();

    max_lidar_distance = this->get_parameter("max_lidar_distance").as_double();
    weight_speed = this->get_parameter("weight_speed").as_double();
    weight_steering = this->get_parameter("weight_steering").as_double();

    // Initialize subscribers and publishers
    lidar_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
        lidarscan_topic, 10, 
        std::bind(&ReactiveFollowerNode::lidar_callback, this, std::placeholders::_1));

    goal_subscriber_ = create_subscription<interfaces_pkg::msg::GoalPoint>(
        goalpoint_topic, 10, 
        std::bind(&ReactiveFollowerNode::goal_callback, this, std::placeholders::_1));

    drive_publisher_ =  create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic, 10);
    
    lidar_publisher_ =  create_publisher<sensor_msgs::msg::LaserScan>(
        "/reactive/laser", 10);

    start_angle = (lidar_angle_front_car - (processed_angle / 2) ) ;  // get start and end angles
    end_angle = (lidar_angle_front_car + (processed_angle / 2) ) ;

    start_index = std::max(0, std::min(lidar_scans, static_cast<int>(start_angle / ((lidar_angle / lidar_scans) ))));   //get start and end indices
    end_index = std::max(0, std::min(lidar_scans, static_cast<int>(end_angle / ((lidar_angle / lidar_scans)))));

    lidar_angle_rad = lidar_angle * (M_PI / 180.0);
    processed_angle_rad = processed_angle * (M_PI / 180.0);
    lidar_angle_front_car_rad = lidar_angle_front_car * (M_PI / 180.0);
    max_steering_angle_rad = max_steering_angle * (M_PI / 180.0);

    // Initialize transform buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "<lidarscan_topic>: %s", lidarscan_topic.c_str());
    RCLCPP_INFO(get_logger(), "<drive_topic>: %s", drive_topic.c_str());
    RCLCPP_INFO(get_logger(), "<goalpoint_topic>: %s", goalpoint_topic.c_str());
    RCLCPP_INFO(get_logger(), "<laser_frame>: %s", laser_frame.c_str());
    RCLCPP_INFO(get_logger(), "<car_frame>: %s", car_frame.c_str());

    RCLCPP_INFO(get_logger(), "<lidar_angle>: %f", lidar_angle);
    RCLCPP_INFO(get_logger(), "<lidar_angle_front_car>: %f", lidar_angle_front_car);
    RCLCPP_INFO(get_logger(), "<lidar_scans>: %d", lidar_scans);

    RCLCPP_INFO(get_logger(), "<max_speed>: %f", max_speed);
    RCLCPP_INFO(get_logger(), "<min_speed>: %f", min_speed);
    RCLCPP_INFO(get_logger(), "<bubble_radius>: %d", bubble_radius);
    RCLCPP_INFO(get_logger(), "<processed_angle>: %f", processed_angle);
    RCLCPP_INFO(get_logger(), "<safety_distance_min>: %f", safety_distance_min);
    RCLCPP_INFO(get_logger(), "<safety_distance_threshold>: %f", safety_distance_threshold);
    RCLCPP_INFO(get_logger(), "<safety_distance_gain>: %f", safety_distance_gain);
    RCLCPP_INFO(get_logger(), "<vehicle_width>: %f", vehicle_width);

    RCLCPP_INFO(get_logger(), "<max_lidar_distance>: %f", max_lidar_distance);
    RCLCPP_INFO(get_logger(), "<weight_speed>: %f", weight_speed);
    RCLCPP_INFO(get_logger(), "<weight_steering>: %f", weight_steering);

    RCLCPP_INFO(get_logger(), "Reactive follower initialized");
}

void ReactiveFollowerNode::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    latest_scan_msg_ = scan_msg;
}

void ReactiveFollowerNode::preprocess_lidar(std::vector<float> &ranges) {
    
    float range = 0.0;
    float last_range = 0.0;

    // Filter out readings beyond max distance
    // Nan reading get the last valid measure
    for (size_t i = 0; i < ranges.size(); i++) 
    {
        range = ranges[i];

        if(std::isnan(range))
        {
            ranges[i] = last_range;
        } else{
            last_range = range;
        }
    }
}

int ReactiveFollowerNode::find_closest_point(const std::vector<float> &ranges) {
    int min_index = 1;
    float min_value = std::numeric_limits<float>::max();

    for (size_t i = 0; i < ranges.size(); i++) {
        if (ranges[i] > 0.0 && ranges[i] < min_value && ranges[i] < 1.0) {
            min_value = ranges[i];
            min_index = i;
        }
    }
    return min_index;
}

void ReactiveFollowerNode::eliminate_bubble(std::vector<float> &ranges, int closest_idx, float bubble_radius) {
    size_t bubble_start = (static_cast<size_t>(closest_idx) >= static_cast<size_t>(bubble_radius)) 
        ? static_cast<size_t>(closest_idx) - static_cast<size_t>(bubble_radius) 
        : 0;
    size_t bubble_end = std::min(closest_idx + static_cast<size_t>(bubble_radius), ranges.size() - 1);
    std::fill(ranges.begin() + bubble_start, ranges.begin() + bubble_end + 1, 0.0);
}

// returns safety distance based on speed
double ReactiveFollowerNode::calculate_safety_distance(double speed){
    // if (speed < safety_distance_threshold) {
    //     RCLCPP_INFO(this->get_logger(), "Safety distance for low speed: %f", safety_distance_min);
    //     return safety_distance_min; // Safety distance for low speeds
    // } else {
    //     RCLCPP_INFO(this->get_logger(), "Safety distance for high speed: %f", safety_distance_min + (speed) * safety_distance_gain);
    //     return safety_distance_min + (speed) * safety_distance_gain; // Proportional increase for higher speeds
    // }
    return safety_distance_min;
}
    // Calculate the minimum number of LiDAR beams for a safe gap
size_t ReactiveFollowerNode::calculate_min_gap_size(double safety_distance) {
    double alpha = 2*(atan2((vehicle_width/ 2), safety_distance)); //geometry cacl
    size_t min_gap = static_cast<size_t>(std::ceil(alpha * (lidar_scans / lidar_angle_rad))); // Number of scans in alpha radians (1180 scans in 270º)
    RCLCPP_INFO(this->get_logger(), "Calculated min gap size: %zu", min_gap);
    return min_gap;
}

// Converts the goal point from the car's frame to the LiDAR frame and computes the corresponding LiDAR scan index.
// This involves transforming the goal coordinates using the latest available transform, calculating the angle to the goal,
// and mapping that angle to the appropriate LiDAR scan index.
size_t ReactiveFollowerNode::point_to_lidar_index(){
    
    double x = goal_msg_->x;
    double y = goal_msg_->y;

    //RCLCPP_INFO(this->get_logger(), "Goal point in car frame: x = %f, y = %f", x, y);

    x = goal_msg_->x - 0.38;

    // Calculate the angle from the x and y coordinates
    double angle = atan2(y, x);
    //RCLCPP_INFO(this->get_logger(), "Angle to goal point: %f", angle);

    // angle += (processed_angle_rad / 2); // Adjust angle to match LiDAR frame (0 rad = front of the car)
    // RCLCPP_INFO(this->get_logger(), "Adjusted angle to LiDAR frame: %f", angle);
    angle = goal_msg_->s + (processed_angle_rad / 2); // Adjust angle to match LiDAR frame (0 rad = front of the car)

    int idx = std::round(angle * lidar_scans/lidar_angle_rad);
    RCLCPP_INFO(this->get_logger(), "Angle: %f, Index: %i", angle, idx);
    // double lidar_clamp = lidar_scans - 1;
    return std::clamp(idx, 0, 1179);
}    

std::pair<size_t, size_t> ReactiveFollowerNode::find_gap(const std::vector<float> &ranges, size_t min_gap) {
    // Find the largest gap in the LiDAR scan data based on the safety distance and minimum gap size
    bool in_gap = false;
    size_t start = 0;
    size_t max_gap_size = 0;
    std::pair<size_t, size_t> biggest_gap{0, 0};

    for (size_t i = 0; i < ranges.size(); ++i) {
        if (ranges[i] > 0.5) {
            if (!in_gap) { start = i; in_gap = true; }
        } else if (in_gap) {
            size_t end = i - 1;
            size_t gap_size = end - start + 1;
            if (gap_size >= min_gap && gap_size > max_gap_size) {
                max_gap_size = gap_size;
                biggest_gap = {start, end};
            }
            in_gap = false;
        }
    }
    // Handle gap that extends to the end of the array
    if (in_gap) {
        size_t end = ranges.size() - 1;
        size_t gap_size = end - start + 1;
        if (gap_size >= min_gap && gap_size > max_gap_size) {
            max_gap_size = gap_size;
            biggest_gap = {start, end};
        }
    }

    if (max_gap_size > 0) {
        // RCLCPP_INFO(this->get_logger(), "Found biggest gap: start=%zu, end=%zu, size=%zu", 
        //            biggest_gap.first, biggest_gap.second, max_gap_size);
    } else {
        RCLCPP_WARN(this->get_logger(), "No valid gaps found with min size %zu", min_gap);
    }

    return biggest_gap;
}

bool ReactiveFollowerNode::gp_in_gaps(const std::vector<Gap> &gaps, const std::vector<float> &ranges) {
    // Check if gp_index has minimum gap size on both left and right sides
    // with distance greater than safety distance
    int half_gap = min_gap_size / 2;
    int left_check_start = gp_index - half_gap;
    int right_check_end = gp_index + half_gap;
    
    // Check bounds
    if (left_check_start < 0 || right_check_end >= static_cast<int>(ranges.size())) {
        RCLCPP_INFO(this->get_logger(), "Goal point out of bounds: left_start=%d, right_end=%d, ranges_size=%zu", 
                   left_check_start, right_check_end, ranges.size());
        return false;
    }
    
    // Check if all points from left half to goal point have sufficient distance
    for (int i = left_check_start; i < static_cast<int>(gp_index); ++i) {
        if (ranges[i] <= safety_distance) {
            RCLCPP_INFO(this->get_logger(), "Left side not safe: index=%d, distance=%f, safety_distance=%f", 
                       i, ranges[i], safety_distance);
            return false;
        }
    }
    
    // Check if all points from goal point to right half have sufficient distance
    for (int i = static_cast<int>(gp_index); i <= right_check_end; ++i) {
        if (ranges[i] <= 0.5) {
            RCLCPP_INFO(this->get_logger(), "Right side not safe: index=%d, distance=%f, safety_distance=%f", 
                       i, ranges[i], safety_distance);
            return false;
        }
    }
    
    // RCLCPP_INFO(this->get_logger(), "Goal point is safe: gp_index=%zu, left_range=[%d,%d], right_range=[%d,%d]", 
    //            gp_index, left_check_start, static_cast<int>(gp_index)-1, static_cast<int>(gp_index), right_check_end);
    return true;
}

std::pair<double, double> ReactiveFollowerNode::alternative_commands(
    const std::pair<size_t, size_t>& biggest_gap, 
    const std::vector<float>& ranges) 
{
    // Check if gap is valid
    if (biggest_gap.first == biggest_gap.second && biggest_gap.first == 0) {
        // No valid gap found, fallback
        RCLCPP_WARN(this->get_logger(), "No valid gap found, stopping");
        double steering_angle_alternativo = 0.0;
        double speed_alternativo = min_speed;
        return std::make_pair(steering_angle_alternativo, speed_alternativo);
    }

    // Place best_idx at the center of the biggest gap
    int best_idx = static_cast<int>(biggest_gap.first + (biggest_gap.second - biggest_gap.first) / 2);

    // RCLCPP_INFO(this->get_logger(), "Targeting center of biggest gap: start=%zu, end=%zu, center_idx=%d", 
    //            biggest_gap.first, biggest_gap.second, best_idx);

    // Convert index to angle
    double best_angle = best_idx * (lidar_angle_rad / lidar_scans);
    double steering_angle_alternativo = (best_angle - (processed_angle_rad / 2)); // Adjust angle to match LiDAR frame
    
    // Limit steering angle to maximum allowed
    steering_angle_alternativo = std::clamp(steering_angle_alternativo, -max_steering_angle_rad, max_steering_angle_rad);
    
    // Calculate speed based on distance and steering angle
    double speed_alternativo = std::min(std::abs(max_speed * (weight_speed * ranges[best_idx] - weight_steering * std::abs(steering_angle_alternativo))), max_speed);
    speed_alternativo = std::clamp(speed_alternativo, min_speed, max_speed);
    
    return std::make_pair(steering_angle_alternativo, speed_alternativo);
}

void ReactiveFollowerNode::publishLaser(const std::vector<float> &ranges) {
    // Publish cropped LaserScan
    sensor_msgs::msg::LaserScan cropped_scan;
    cropped_scan.header = latest_scan_msg_->header;
    cropped_scan.header.stamp = this->now();  // update timestamp
    cropped_scan.angle_min = latest_scan_msg_->angle_min + start_index * latest_scan_msg_->angle_increment;
    cropped_scan.angle_max = latest_scan_msg_->angle_min + end_index * latest_scan_msg_->angle_increment;
    cropped_scan.angle_increment = latest_scan_msg_->angle_increment;
    cropped_scan.time_increment = latest_scan_msg_->time_increment;
    cropped_scan.scan_time = latest_scan_msg_->scan_time;
    cropped_scan.range_min = latest_scan_msg_->range_min;
    cropped_scan.range_max = latest_scan_msg_->range_max;
    cropped_scan.ranges = ranges;
    cropped_scan.intensities = std::vector<float>(ranges.size(), 0.0);  // Fill with zeros or actual intensities if you have them

    lidar_publisher_->publish(cropped_scan);
}


void ReactiveFollowerNode::goal_callback(const interfaces_pkg::msg::GoalPoint::ConstSharedPtr goal_msg) {

    goal_msg_ = goal_msg; // store

    if (!latest_scan_msg_) {
    RCLCPP_WARN(get_logger(), "No LiDAR data yet, skipping goal");
    return;
    }

    std::vector<float> ranges = latest_scan_msg_->ranges;
    std::vector<float> cropped_ranges(end_index - start_index + 1);

    // Get only the front section
    for (size_t i = 0; i < cropped_ranges.size(); ++i) {
        cropped_ranges[i] = ranges[i + start_index];
    }

    preprocess_lidar(cropped_ranges);
    int closest_idx = find_closest_point(cropped_ranges);
    eliminate_bubble(cropped_ranges, closest_idx, bubble_radius);

    publishLaser(cropped_ranges);

    double speed = goal_msg->v;
    double steering_angle = goal_msg->s;

    safety_distance = calculate_safety_distance(speed);
    min_gap_size = calculate_min_gap_size(safety_distance);
    gp_index = point_to_lidar_index();

    // Check if goal point is safe using the new logic
    bool is_goal_safe = gp_in_gaps(std::vector<Gap>(), cropped_ranges);  // We don't need gaps for the new logic
    
    if (is_goal_safe) {
        // Goal point is safe, use pure pursuit commands
        RCLCPP_INFO(get_logger(), "Pure Pursuit");
        steering_angle = std::clamp(steering_angle, -max_steering_angle_rad, max_steering_angle_rad);
    } else {
        // Goal point is not safe, find biggest gap and drive to it
        RCLCPP_INFO(get_logger(), "Alternative commands");
        auto biggest_gap = find_gap(cropped_ranges, min_gap_size);
        auto [alt_steering_angle, alt_speed] = alternative_commands(biggest_gap, cropped_ranges);
        steering_angle = -alt_steering_angle;
        speed = alt_speed;
    }

    //RCLCPP_INFO(get_logger(), "close index: %zu", closest_idx);
    RCLCPP_INFO(get_logger(), "Steering angle: %f", steering_angle);
    //RCLCPP_INFO(get_logger(), "\n");

    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.drive.speed = std::max(speed, min_speed);
    drive_msg.drive.steering_angle = steering_angle;
    drive_publisher_->publish(drive_msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowerNode>());
    rclcpp::shutdown();
    return 0;
}