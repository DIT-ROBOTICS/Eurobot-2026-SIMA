#ifndef VL53_BRIDGE_HPP_
#define VL53_BRIDGE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 

#include <vector>
#include <string>
#include <cmath>

class VL53Bridge : public rclcpp::Node
{
public:
    VL53Bridge();

private:
    // Sensor configuration structure
    struct SensorConfig {
        std::string name;
        double x_offset;   // relative to base_link
        double y_offset;   // relative to base_link
        double yaw_angle;  // installation angle (radians)
    };

    void rawDataCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    
    // Parameter declaration and loading
    void loadParameters();

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_raw_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_obstacles_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    std::vector<SensorConfig> sensors_;
    double trigger_distance_ = 0.4; // less than this distance is considered an obstacle
    double min_valid_dist_ = 0.05;  // minimum valid distance (to filter noise in order to avoid affecting by itself)
};

#endif // VL53_BRIDGE_HPP_