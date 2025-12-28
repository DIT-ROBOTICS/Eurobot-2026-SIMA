#include "sima-localization-sim/robot-pose_node.hpp"

RobotPoseNode::RobotPoseNode()
    : Node("robot_pose_node"), x_(0.0), y_(0.0), theta_(0.0)
{
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&RobotPoseNode::cmd_vel_callback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    last_time_ = this->now();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&RobotPoseNode::update_pose, this));
}

void RobotPoseNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    current_cmd_vel_ = *msg;
}

void RobotPoseNode::update_pose()
{
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();

    // Update pose based on current velocities
    double delta_x = current_cmd_vel_.linear.x * dt * cos(theta_);
    double delta_y = current_cmd_vel_.linear.x * dt * sin(theta_);
    double delta_theta = current_cmd_vel_.angular.z * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    // Create and send the transform
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = current_time;
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "robot_base";
    transformStamped.transform.translation.x = x_;
    transformStamped.transform.translation.y = y_;
    transformStamped.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transformStamped);

    last_time_ = current_time;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotPoseNode>());
    rclcpp::shutdown();
    return 0;
}