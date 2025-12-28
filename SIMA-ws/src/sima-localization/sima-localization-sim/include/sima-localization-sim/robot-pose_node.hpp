#ifndef ROBOT_POSE_NODE_HPP
#define ROBOT_POSE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class RobotPoseNode : public rclcpp::Node
{
public:
    RobotPoseNode();

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void update_pose();

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Robot pose state
    double x_, y_, theta_;
    rclcpp::Time last_time_;
    geometry_msgs::msg::Twist current_cmd_vel_;
};

#endif // ROBOT_POSE_NODE_HPP