#include "sima-localization-sim/odom-sim_node.hpp"

OdomSimNode::OdomSimNode()
    : Node("odom_sim_node"), x_(0.0), y_(0.0), theta_(0.0),
      gen_(rd_()), slip_dist_(-0.05, 0.05) // Slip noise between -0.05 and 0.05 m/s or rad/s
{
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&OdomSimNode::cmd_vel_callback, this, std::placeholders::_1));

    last_time_ = this->now();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&OdomSimNode::timer_callback, this));
}
void OdomSimNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    current_cmd_vel_ = *msg;
}
void OdomSimNode::add_slip_noise(double& vx, double& vy, double& omega)
{
    vx += slip_dist_(gen_);
    vy += slip_dist_(gen_);
    omega += slip_dist_(gen_);
}
void OdomSimNode::timer_callback()
{
    rclcpp::Time current_time = this->now();
    double vx = current_cmd_vel_.linear.x;
    double vy = current_cmd_vel_.linear.y;
    double omega = current_cmd_vel_.angular.z;
    
    // Add slip noise
    add_slip_noise(vx, vy, omega);
    
    // Prepare odometry message (velocity only)
    odom_msg_.header.stamp = current_time;
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "robot_base";
    
    // Set position to zero/default
    odom_msg_.pose.pose.position.x = 0.0;
    odom_msg_.pose.pose.position.y = 0.0;
    odom_msg_.pose.pose.position.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    
    // Set velocity data
    odom_msg_.twist.twist.linear.x = vx;
    odom_msg_.twist.twist.linear.y = vy;        
    odom_msg_.twist.twist.angular.z = omega;
    
    // Publish odometry
    odom_pub_->publish(odom_msg_);
    last_time_ = current_time;
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomSimNode>());
    rclcpp::shutdown();
    return 0;
}
