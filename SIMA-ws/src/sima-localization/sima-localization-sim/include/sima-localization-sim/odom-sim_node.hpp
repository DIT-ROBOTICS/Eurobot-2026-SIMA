#ifndef SIMA_LOCALIZATION_SIM_ODOM_SIM_NODE_HPP
#define SIMA_LOCALIZATION_SIM_ODOM_SIM_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <random>


class OdomSimNode : public rclcpp::Node {
public:
    OdomSimNode();

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void timer_callback();
    void add_slip_noise(double& vx, double& vy, double& omega);

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist current_cmd_vel_;
    nav_msgs::msg::Odometry odom_msg_;
    
    // Pose tracking
    double x_, y_, theta_;
    rclcpp::Time last_time_;

    // Noise generation
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> slip_dist_;
};


#endif // SIMA_LOCALIZATION_SIM_ODOM_SIM_NODE_HPP