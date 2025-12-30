#include "sima-localization-sim/global-sim_node.hpp"

GlobalSimNode::GlobalSimNode()
    : Node("global_sim_node"), 
      gen_(rd_()),
      linear_noise_dist_(-0.05, 0.05),      // ±5cm
      rotation_noise_dist_(-0.05, 0.05),    // ±0.05 rad
      max_linear_noise_(0.05),
      max_rotation_noise_(0.05)
{
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "pose/global", 10);

    // Initialize TF2 buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GlobalSimNode::timer_callback, this));
}

void GlobalSimNode::timer_callback()
{
    auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
    message.header.stamp = this->now();
    message.header.frame_id = "map";

    // Get theoretical position from transform
    if (get_theoretical_pose(message))
    {
        // Add AprilTag sensor noise
        add_noise_to_pose(message);
        publisher_->publish(message);
    }
}

bool GlobalSimNode::get_theoretical_pose(geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg)
{
    try
    {
        // Look up the transform from map to robot_base
        geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
            "world", "robot_base", tf2::TimePointZero);

        // Convert transform to pose
        pose_msg.pose.pose.position.x = transform_stamped.transform.translation.x;
        pose_msg.pose.pose.position.y = transform_stamped.transform.translation.y;
        pose_msg.pose.pose.position.z = transform_stamped.transform.translation.z;
        
        pose_msg.pose.pose.orientation = transform_stamped.transform.rotation;
        
        return true;
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform map to robot_base: %s", ex.what());
        return false;
    }
}

void GlobalSimNode::add_noise_to_pose(geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg)
{
    // Add linear noise to x and y positions
    pose_msg.pose.pose.position.x += linear_noise_dist_(gen_);
    pose_msg.pose.pose.position.y += linear_noise_dist_(gen_);
    
    // Add rotational noise to orientation
    double yaw_noise = rotation_noise_dist_(gen_);
    
    // Convert current quaternion to yaw, add noise, convert back
    double current_yaw = atan2(2.0 * (pose_msg.pose.pose.orientation.w * pose_msg.pose.pose.orientation.z),
                              1.0 - 2.0 * pose_msg.pose.pose.orientation.z * pose_msg.pose.pose.orientation.z);
    
    double noisy_yaw = current_yaw + yaw_noise;
    
    // Convert back to quaternion
    pose_msg.pose.pose.orientation.w = cos(noisy_yaw / 2.0);
    pose_msg.pose.pose.orientation.x = 0.0;
    pose_msg.pose.pose.orientation.y = 0.0;
    pose_msg.pose.pose.orientation.z = sin(noisy_yaw / 2.0);
    // Set covariance matrix for the pose
    // Initialize all covariance values to zero first
    std::fill(pose_msg.pose.covariance.begin(), pose_msg.pose.covariance.end(), 0.0);
    
    // Diagonal values represent variance (std_dev^2) for x, y, z, roll, pitch, yaw
    pose_msg.pose.covariance[0] = max_linear_noise_ * max_linear_noise_;   // x variance
    pose_msg.pose.covariance[7] = max_linear_noise_ * max_linear_noise_;   // y variance
    pose_msg.pose.covariance[14] = 1e-6;                              // z variance (small but non-zero for 2D)
    pose_msg.pose.covariance[21] = 1e-6;                              // roll variance (small but non-zero for 2D)
    pose_msg.pose.covariance[28] = 1e-6;                              // pitch variance (small but non-zero for 2D)
    pose_msg.pose.covariance[35] = max_rotation_noise_ * max_rotation_noise_; // yaw variance
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalSimNode>());
    rclcpp::shutdown();
    return 0;
}