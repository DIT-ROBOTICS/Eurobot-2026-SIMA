#ifndef SPECIAL_DIFF_CONTROLLER__SPECIAL_DIFF_CONTROLLER_HPP_
#define SPECIAL_DIFF_CONTROLLER__SPECIAL_DIFF_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace special_diff_controller
{

enum class DriveState {
    ROTATING,
    DRIVING
};

class SpecialDiffController : public nav2_core::Controller
{
public:
    SpecialDiffController() = default;
    ~SpecialDiffController() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;
    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override;

    void setPlan(const nav_msgs::msg::Path & path) override;

protected:
    nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);
    
    bool transformPose(
        const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::string & frame,
        const geometry_msgs::msg::PoseStamped & in_pose,
        geometry_msgs::msg::PoseStamped & out_pose,
        const rclcpp::Duration & transform_tolerance) const;

    // Core Components
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    rclcpp::Logger logger_ {rclcpp::get_logger("SpecialDiffController")};
    rclcpp::Clock::SharedPtr clock_;
    std::string plugin_name_;

    // Plan & State
    nav_msgs::msg::Path global_plan_;
    DriveState current_state_;

    // Slew Rate Limits (Memory)
    double last_linear_vel_;
    double last_angular_vel_;
    rclcpp::Time last_time_;

    // Publishers for debug
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr debug_global_plan_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_lookahead_pub_;

    // Parameters
    double desired_linear_vel_;
    double lookahead_dist_;
    double max_angular_vel_;
    double min_approach_linear_vel_;
    double approach_dist_;
    
    // State Machine Thresholds
    double heading_error_threshold_to_rotate_;
    double heading_error_threshold_to_drive_;
    
    // Kp settings
    double heading_kp_;
    double straight_correction_kp_;

    // Accel / Decel Limits
    double max_acc_linear_;
    double max_acc_angular_;
    double max_decel_linear_;
    double max_decel_linear_emergency_;
    double max_decel_angular_;

    rclcpp::Duration transform_tolerance_{0, 0};
};

}  // namespace special_diff_controller

#endif  // SPECIAL_DIFF_CONTROLLER__SPECIAL_DIFF_CONTROLLER_HPP_