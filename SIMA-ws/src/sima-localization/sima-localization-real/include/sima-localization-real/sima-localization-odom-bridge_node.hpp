#ifndef SIMA_LOCALIZATION_ODOM_BRIDGE_NODE_HPP
#define SIMA_LOCALIZATION_ODOM_BRIDGE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>


class SimaLocalizationOdomBridgeNode : public rclcpp::Node
{
public:
    SimaLocalizationOdomBridgeNode();
    
private:
    // Member variables here
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_bridge_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_raw_sub_;
    // Member functions here
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};


#endif // SIMA_LOCALIZATION_ODOM_BRIDGE_NODE_HPP