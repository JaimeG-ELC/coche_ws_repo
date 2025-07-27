#include "pure_pursuit_pkg/pure_pursuit_node.hpp"

PurePursuit::PurePursuit() : Node("pure_pursuit_node")
{
    // Establish some private variables as parameters
    this->declare_parameter<double>("min_lookahead_dist", 0.5);
    this->declare_parameter<double>("max_lookahead_dist", 4.0);
    this->declare_parameter<double>("lookahead_ratio", 8.0);
    this->declare_parameter<double>("max_speed", 4.0);
    this->declare_parameter<double>("min_speed", 0.4); // Default minimum speed if not specified
    this->declare_parameter<double>("Kp", 0.25);
    this->declare_parameter<double>("max_steering_angle", 44); //grados
    this->declare_parameter<int>("window_size", 25);
    this->declare_parameter<double>("max_lateral_acc", 5.0);
    this->declare_parameter<std::string>("csv_path", "/root/coche_ws/src/pure_pursuit_pkg/racelines/pathpoints_odom_3.csv");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("car_frame", "base_link");
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("goalpoint_topic", "/goalpoint");
    this->declare_parameter<std::string>("drive_topic", "/drive");
    this->declare_parameter<bool>("reactive", true); // Default to false if not specified

    // Retrieve parameter values
    min_lookahead_dist = this->get_parameter("min_lookahead_dist").as_double();
    max_lookahead_dist = this->get_parameter("max_lookahead_dist").as_double();
    lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
    max_speed = this->get_parameter("max_speed").as_double();
    min_speed = this->get_parameter("min_speed").as_double();
    Kp = this->get_parameter("Kp").as_double();
    max_steering_angle = this->get_parameter("max_steering_angle").as_double();
    window_size = this->get_parameter("window_size").as_int();
    max_lateral_acc = this->get_parameter("max_lateral_acc").as_double();
    csv_path = this->get_parameter("csv_path").as_string();
    map_frame = this->get_parameter("map_frame").as_string();
    car_frame = this->get_parameter("car_frame").as_string();
    odom_topic = this->get_parameter("odom_topic").as_string();
    goalpoint_topic  = this->get_parameter("goalpoint_topic").as_string();
    drive_topic = this->get_parameter("drive_topic").as_string();
    reactive = this->get_parameter("reactive").as_bool();

    RCLCPP_INFO(this->get_logger(), "Pure Pursuit Node has started.");
    RCLCPP_INFO(this->get_logger(), "CSV Path: %s", csv_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Odom Topic: %s", odom_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Goal Point Topic: %s", goalpoint_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Drive Topic: %s", drive_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Map Frame: %s", map_frame.c_str());    
    RCLCPP_INFO(this->get_logger(), "Car Frame: %s", car_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "Minimum Lookahead Distance: %f", min_lookahead_dist);
    RCLCPP_INFO(this->get_logger(), "Maximum Lookahead Distance: %f", max_lookahead_dist);  
    RCLCPP_INFO(this->get_logger(), "Lookahead Ratio: %f", lookahead_ratio);
    RCLCPP_INFO(this->get_logger(), "Max Speed: %f", max_speed);
    RCLCPP_INFO(this->get_logger(), "Min Speed: %f", min_speed);
    RCLCPP_INFO(this->get_logger(), "Kp: %f", Kp);
    RCLCPP_INFO(this->get_logger(), "Max Steering Angle: %f", max_steering_angle);
    RCLCPP_INFO(this->get_logger(), "Window Size: %d", window_size);
    RCLCPP_INFO(this->get_logger(), "Max Lateral Acceleration: %f", max_lateral_acc);
    RCLCPP_INFO(this->get_logger(), "Reactive Mode: %s", reactive ? "true" : "false");

    // Other required member variables
    graph_topic = "visualization_marker";

    closest_pathpoint = 0;
    lookahead_point = 0;
    lookahead_dist = 0.0;
    steering_angle = 0.0;
    speed = 0.0;
    n_pathpoints = 0;
    start_index = 0;

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 100,
        std::bind(&PurePursuit::odom_callback, this, std::placeholders::_1));
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic, 10);
    goal_pub_ = this->create_publisher<interfaces_pkg::msg::GoalPoint>(
        goalpoint_topic, 10);    
    graph_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(graph_topic, 10);

    // Buffer para guardar Transformaciones entre Coordinate Frames
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    // Creamos un objeto de tipo Listener para que automáticamente guarde la Transformación en el Buffer
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Calculate the number of pathpoints from the CSV file
    n_pathpoints = calculate_n_pathpoints(csv_path);

    if (window_size > n_pathpoints) {
        RCLCPP_ERROR(this->get_logger(), "Window size (%d) larger than path points (%d)", window_size, n_pathpoints);
        return;
    }

    max_steering_angle = to_radians(max_steering_angle); // Convert to radians

    // We load the path into memory
    load_pathpoints2memory();
}

double PurePursuit::to_radians(double degrees) {
    double radians;
    return radians = degrees * M_PI / 180.0;
}

double PurePursuit::to_degrees(double radians) {
    double degrees;
    return degrees = radians * 180.0 / M_PI;
}

double PurePursuit::p2pdist(double x1, double x2, double y1, double y2) 
{
    double dist = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
    return dist;
}

Eigen::Matrix3d PurePursuit::quaternionToMatrix(const geometry_msgs::msg::Quaternion& q)
{
    // Matrix will represent R_car2map (car to map rotation)
    double q0 = q.w;
    double q1 = q.x;
    double q2 = q.y;
    double q3 = q.z;

    Eigen::Matrix3d R_car2map;
    R_car2map << 2 * (q0*q0 + q1*q1) - 1,
         2 * (q1*q2 - q0*q3),
         2 * (q1*q3 + q0*q2),
         2 * (q1*q2 + q0*q3),
         2 * (q0*q0 + q2*q2) - 1,
         2 * (q2*q3 - q0*q1),
         2 * (q1*q3 - q0*q2),
         2 * (q2*q3 + q0*q1),
         2 * (q0*q0 + q3*q3) - 1;
    return R_car2map;
}

Eigen::Vector3d PurePursuit::transform_to_car_frame(const Eigen::Vector3d& point_map)
{
    // Use cached transform instead of looking it up again
    Eigen::Vector3d t_car_in_map(
        current_transform_.transform.translation.x,
        current_transform_.transform.translation.y,
        current_transform_.transform.translation.z
    );

    // Get car→map rotation and transpose for map→car
    Eigen::Matrix3d R_car2map = quaternionToMatrix(current_transform_.transform.rotation);
    Eigen::Matrix3d R_map2car = R_car2map.transpose();
    
    // First subtract translation, then rotate
    return R_map2car * (point_map - t_car_in_map);
}

void PurePursuit::map2car()
{
    v_local = transform_to_car_frame(v_global);
}

int PurePursuit::calculate_n_pathpoints(const std::string& csv_path)
{
    // Open the csv file
    std::ifstream csv(csv_path);
    if (!csv.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Error: Could Not Open the File %s", csv_path.c_str());
        return -1;
    }

    // Count the number of lines in the file
    int count = 0;
    std::string line;
    while (std::getline(csv, line)) {
        if (!line.empty()) {
            count++;
        }
    }

    csv.close();
    RCLCPP_INFO(this->get_logger(), "Number of pathpoints: %d", count);
    return count;
}

int PurePursuit::load_pathpoints2memory()
{
    // Open the csv
    std::ifstream csv(csv_path);

    if(!csv.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Error: Could Not Open the File %s", csv_path.c_str());
        return -1;
    }

    // Create a vector to hold PathPoints
    pathpoints.reserve(n_pathpoints);

    std::string row, x_str, y_str, v_str;

    for(int i = 0; i < n_pathpoints; i++)
    {
        // Read one line (x, y, v)
        std::getline(csv, row, '\n');
        std::stringstream ss(row);

        // Extract x, y, v (v may be missing)
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');
        if (!std::getline(ss, v_str)) {
            v_str = ""; // v is missing
            RCLCPP_WARN(this->get_logger(), "Missing velocity for point %d, defaulting to min_speed", i);
        }

        double x = std::stod(x_str);
        double y = std::stod(y_str);
        double v;
        if (v_str.empty()) {
            v = min_speed; // Default to min_speed if v is missing
        } else {
            try {
                v = std::stod(v_str);
            } catch (...) {
                v = min_speed;
            }
        }

        // Push the new element into the vector
        pathpoints.emplace_back(x, y, v);
    }

    return 0;
}

void PurePursuit::graph_closest_pathpoint()
{
    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = map_frame;  // Use parameter instead of hardcoded "map"
    marker.header.stamp = this->now();  

    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::msg::Marker::SPHERE;

    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color.a = 1.0;
    marker.color.r = 1.0;

    marker.pose.position.x = pathpoints[closest_pathpoint].x;
    marker.pose.position.y = pathpoints[closest_pathpoint].y;
    marker.pose.position.z = 0.0;

    // Add logging for waypoint information
    // RCLCPP_INFO(this->get_logger(), "Using waypoint %d at position (%.2f, %.2f)", 
                // start_index, v_global[0], v_global[1]);

    graph_pub_->publish(marker);
}

void PurePursuit::graph_lookahead_point()
{
    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = map_frame;  // Use parameter instead of hardcoded "map"
    marker.header.stamp = this->now();

    marker.ns = "basic_shapes";
    marker.id = 1;

    marker.type = visualization_msgs::msg::Marker::SPHERE;

    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color.a = 1.0;
    marker.color.g = 1.0;

    marker.pose.position.x = pathpoints[lookahead_point].x;
    marker.pose.position.y = pathpoints[lookahead_point].y;
    marker.pose.position.z = 0.0;

    // Add logging for waypoint information
    // RCLCPP_INFO(this->get_logger(), "Using waypoint %d at position (%.2f, %.2f)", 
                // start_index, v_global[0], v_global[1]);

    graph_pub_->publish(marker);
}

void PurePursuit::get_lookahead_point()
{
    int best_i = -1;
    double closest_distance = std::numeric_limits<double>::max();

    // Search in the [start_index ... start_index + window_size) window
    for (int n = 0; n < window_size; ++n) {
        int i = (start_index + n) % n_pathpoints;

        // 1) Distance in map frame
        double dx = pathpoints[i].x - curr_pose.x;
        double dy = pathpoints[i].y - curr_pose.y;
        double d  = std::hypot(dx, dy);

        // 2) Skip if it's too close or too far
        if (d < lookahead_dist || d >= closest_distance) {
            continue;
        }

        // 3) Check that the point is in front by transforming to car frame
        Eigen::Vector3d p_map(pathpoints[i].x, pathpoints[i].y, 0.0);
        Eigen::Vector3d p_car = transform_to_car_frame(p_map);

        if (p_car.x() <= 0.0) {
            // behind the car → skip
            continue;
        }

        // 4) This is our new best candidate
        closest_distance = d;
        best_i           = i;
        v_global << pathpoints[i].x, pathpoints[i].y, 0.0;
    }

    if (best_i >= 0) {
        // Found a valid look‑ahead
        lookahead_point = best_i;
        start_index     = best_i;  // slide the window forward
    } else {
        // No valid point: fall back halfway through window
        int fallback = (start_index + window_size/2) % n_pathpoints;
        lookahead_point = fallback;
        start_index     = fallback;
        RCLCPP_WARN(this->get_logger(),
            "No lookahead point ≥ %.2fm in front; fallback to index %d",
            lookahead_dist, fallback);
    }
}




void PurePursuit::steering_angle_calculation()
{
    // wheelbase = 0.33;              // 60 cm between axles
    // front_axle_offset = wheelbase/2.0; // if base_link at centroid
    // Turning radius 
    // middle 0.482
    // 0.1 130cm 1/-1,3 = -0,769 rad = -44º
    // 0.864 145cm
    // 0.9 130cm 1/1,3 = 0,769 rad = 44º
    // min 0.112 max 0.864

    // gain for steering vesc 0,4967 left, 0,54356 right
    // 0,482 - 0,769 rad * 0,4967 = 0,1
    // 0,482 + 0,769 rad * 0,54356 = 0,9

    double distance_squared = v_local[0] * v_local[0] + v_local[1] * v_local[1];
    
    // Protect against division by zero
    if (distance_squared < 1e-6) {
        steering_angle = 0.0;
        return;
    }
    
    double distance = std::sqrt(distance_squared);
    double angle = Kp * (2 * v_local[1]) / (distance * distance);
    
    steering_angle = std::clamp(angle, -max_steering_angle, max_steering_angle);
}


void PurePursuit::get_closest_pathpoint()
{
    // Find the closest point to the car, and use the velocity index for that
    int start_point = std::max(start_index - (window_size / 3), 0);
    double shortest_distance = p2pdist(pathpoints[start_point].x, curr_pose.x, pathpoints[start_point].y, curr_pose.y);
    int speed_i = start_point;

    // Use a separate loop variable for iteration
    for (int i = start_point; i < (start_point + window_size); i++) 
    {
        double distance = p2pdist(pathpoints[i].x, curr_pose.x, pathpoints[i].y, curr_pose.y);
        if (distance <= shortest_distance) 
        {
            shortest_distance = distance;
            speed_i = i;
        }
    }
    closest_pathpoint = speed_i;
}

void PurePursuit::speed_calculation()
{
    // Base speed from path
    double target_speed = pathpoints[closest_pathpoint].v;
    
    // Adjust speed based on steering angle magnitude
    if (std::abs(steering_angle) > 0.05) { // About 2.86 degrees

        double radius = 1 / std::abs(steering_angle); 
        double curve_speed = std::sqrt(max_lateral_acc / std::abs(steering_angle));
        target_speed = std::min(target_speed, curve_speed);
        RCLCPP_INFO(this->get_logger(), "Max curve speed: %f", curve_speed);
    }
    // Clamp to speed limits
    speed = std::clamp(target_speed, min_speed, max_speed);
}

void PurePursuit::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
    // Get forward speed from odom
    double curr_vel = std::hypot(
        odom_msg->twist.twist.linear.x,
        odom_msg->twist.twist.linear.y
    );

    // Calculate lookahead distance based on current speed
    lookahead_dist = std::min(std::max(min_lookahead_dist, max_lookahead_dist * curr_vel / lookahead_ratio), max_lookahead_dist);

    // Cache current transform for this cycle
    try {
        current_transform_ = tf_buffer_->lookupTransform(
            map_frame,          // target frame
            car_frame,          // source frame
            tf2::TimePointZero, 
            std::chrono::milliseconds(100)
        );
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(
            this->get_logger(),
            "Failed to get %s → %s transform: %s",
            map_frame.c_str(), car_frame.c_str(), ex.what()
        );
        return;
    }

    // Update current pose
    curr_pose.x = current_transform_.transform.translation.x;
    curr_pose.y = current_transform_.transform.translation.y;

    // Rest of the pipeline uses cached transform

    get_closest_pathpoint();
    graph_closest_pathpoint();

    get_lookahead_point();
    graph_lookahead_point();

    map2car();

    steering_angle_calculation();
    speed_calculation();

    if (reactive){
            // Build a GoalPoint message
        interfaces_pkg::msg::GoalPoint goal;
        // the typical fields might be `x`, `y`, `v` (speed), `s` (steering)
        goal.x = v_local[0];                // global target x
        goal.y = v_local[1];                // global target y
        goal.v = speed;                    // desired speed
        goal.s = steering_angle;                          // desired steering curvature/angle
        goal_pub_->publish(goal);
    }
    else{
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.speed = speed;
        drive_msg.drive.steering_angle = steering_angle;
        RCLCPP_INFO(this->get_logger(), "Steering angle: %f", steering_angle);
        RCLCPP_INFO(this->get_logger(), "Speed: %f", speed);
        drive_pub_->publish(drive_msg);
    } 
}

int main(int argc, char**argv)
{
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<PurePursuit>();
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
}