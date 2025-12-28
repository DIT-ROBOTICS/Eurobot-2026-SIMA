#ifndef GLOBAL_SIM_NODE_HPP
#define GLOBAL_SIM_NODE_HPP

#include <chrono>
#include <memory>
#include <functional>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


class GlobalSimNode : public rclcpp::Node
{
public:
    GlobalSimNode();

private:
    void timer_callback();
    
    // Add noise to pose for AprilTag simulation
    void add_noise_to_pose(geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg);
    
    // Get theoretical position from transform
    bool get_theoretical_pose(geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    
    // TF2 members for getting theoretical position
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Noise simulation members
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> linear_noise_dist_;
    std::uniform_real_distribution<double> rotation_noise_dist_;
    
    // Noise parameters
    double max_linear_noise_;  // 0.05m (5cm)
    double max_rotation_noise_; // 0.05 rad
};

#endif  // GLOBAL_SIM_NODE_HPP
