#include "navigation2_run/vl53_bridge.hpp"

// transform degree to radian
constexpr double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

VL53Bridge::VL53Bridge() : Node("vl53_bridge_node")
{
    // 1. Initialize TF buffer
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 2. load parameters
    loadParameters();

    // 3. Establish communication
    // QoS set to Best Effort because losing one or two frames of sensor data is acceptable; timeliness is more important
    rclcpp::QoS qos_sensor(10);
    qos_sensor.best_effort();
    
    sub_raw_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/sensors/raw_ranges", qos_sensor, 
        std::bind(&VL53Bridge::rawDataCallback, this, std::placeholders::_1));

    pub_obstacles_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/sensors/detected_obstacles", 10);

    RCLCPP_INFO(this->get_logger(), "VL53 Bridge Node Started. Listening for raw ranges...");
}

void VL53Bridge::loadParameters()
{
    // default sensor configurations
    // [0:Left, 1:Center, 2:Right]
    sensors_ = {
        {"Left",   0.075,  0.075, deg2rad(45.0)},
        {"Center", 0.075,  0.0,   deg2rad(0.0)},
        {"Right",  0.075, -0.075, deg2rad(-45.0)}
    };

    // TODO: read param file to override default sensor configs
    this->declare_parameter("trigger_distance", 0.5);
    this->get_parameter("trigger_distance", trigger_distance_);
}

void VL53Bridge::rawDataCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    // 1. check data size
    if (msg->data.size() != sensors_.size()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
            "Received data size (%ld) does not match sensor count (%ld)!", 
            msg->data.size(), sensors_.size());
        return;
    }

    // 2. ready output message
    geometry_msgs::msg::PoseArray output_msg;
    output_msg.header.stamp = this->now();
    output_msg.header.frame_id = "map"; // transform to map frame

    // 3. check if TF is available
    // We need to transform points from base_link to map
    // Here we don't query Transform, but directly use tf_buffer->transform() function
    if (!tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
            "Waiting for TF (map -> base_link)... Cannot publish obstacles.");
        return;
    }

    // 4. iterate each sensor
    for (size_t i = 0; i < sensors_.size(); ++i) {
        float dist = msg->data[i];

        // check if it is a valid obstacle
        if (dist > min_valid_dist_ && dist < trigger_distance_) {
            
            const auto& sensor = sensors_[i];

            // A. calculate obstacle position in robot frame (base_link)
            // Obs_X = installation X + distance * cos(installation angle)
            // Obs_Y = installation Y + distance * sin(installation angle)
            double local_x = sensor.x_offset + dist * std::cos(sensor.yaw_angle);
            double local_y = sensor.y_offset + dist * std::sin(sensor.yaw_angle);

            // B. create PointStamped (to let TF know this point is in base_link)
            geometry_msgs::msg::PointStamped point_in_base, point_in_map;
            point_in_base.header.frame_id = "base_link";
            point_in_base.header.stamp = rclcpp::Time(0); // latest time
            point_in_base.point.x = local_x;
            point_in_base.point.y = local_y;
            point_in_base.point.z = 0.0;

            try {
                // C. perform coordinate transformation (base_link -> map)
                // This line automatically handles the robot's position and orientation
                point_in_map = tf_buffer_->transform(point_in_base, "map");

                // D. add to output list
                geometry_msgs::msg::Pose pose_map;
                pose_map.position = point_in_map.point;
                pose_map.orientation.w = 1.0; // obstacle is a point, orientation is not important
                
                output_msg.poses.push_back(pose_map);

                // Debug
                // RCLCPP_INFO(this->get_logger(), "[%s] Hit at %.2fm -> Map(%.2f, %.2f)", 
                //     sensor.name.c_str(), dist, point_in_map.point.x, point_in_map.point.y);

            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "TF Transform failed: %s", ex.what());
                continue;
            }
        }
    }

    // 5. publish
    // Even if there are no obstacles, consider whether to publish an empty message (depending on your SensorLayer logic)
    // Here we choose: always publish, so the monitoring side knows the node is alive
    pub_obstacles_->publish(output_msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VL53Bridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}