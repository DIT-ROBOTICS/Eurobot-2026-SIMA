#include "sima-localization-real/sima-localization-odom-bridge_node.hpp"


SimaLocalizationOdomBridgeNode::SimaLocalizationOdomBridgeNode()
    : Node("sima_localization_odom_bridge_node") {
    
    // Initialize subscribers
    odom_raw_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "wheel/odom", 10,
        std::bind(&SimaLocalizationOdomBridgeNode::odomCallback, this, std::placeholders::_1));
    
    // Initialize publishers
    odom_bridge_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/wheel", 10);
    
    RCLCPP_INFO(this->get_logger(), "SIMA Localization Odom Bridge Node started");
}

void SimaLocalizationOdomBridgeNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Convert odometry to pose with covariance
    auto pose_msg = std::make_shared<nav_msgs::msg::Odometry>();
    
    pose_msg->header = msg->header;
    pose_msg->header.frame_id = "odom";  // Change frame if necessary
    pose_msg->child_frame_id = "base_link";
    pose_msg->header.stamp = this->now();
    pose_msg->pose.pose = msg->pose.pose;
    pose_msg->pose.covariance = msg->pose.covariance;
    pose_msg->twist.twist = msg->twist.twist;
    pose_msg->twist.covariance = msg->twist.covariance;
    
    odom_bridge_pub_->publish(*pose_msg);
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimaLocalizationOdomBridgeNode>());
    rclcpp::shutdown();
    return 0;
}