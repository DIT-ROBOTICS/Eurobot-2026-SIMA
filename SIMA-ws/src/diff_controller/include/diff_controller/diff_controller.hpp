#ifndef DIFF_CONTROLLER__DIFF_CONTROLLER_HPP_
#define DIFF_CONTROLLER__DIFF_CONTROLLER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace diff_controller
{

class DiffController : public nav2_core::Controller
{
public:
    DiffController() = default;
    ~DiffController() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;
    void setPlan(const nav_msgs::msg::Path & path) override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override;
    
    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
    nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

    bool transformPose(
        const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::string & frame,
        const geometry_msgs::msg::PoseStamped & in_pose,
        geometry_msgs::msg::PoseStamped & out_pose,
        const rclcpp::Duration & transform_tolerance) const;
    
    // ROS2 / Nav2 handles
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::string plugin_name_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_{rclcpp::get_logger("diff_controller")};

    // Parameters
    double desired_linear_vel_{0.4};
    double lookahead_dist_{0.3};
    double max_angular_vel_{2.0};
    double min_linear_vel_{0.0};
    double min_approach_linear_vel_{0.1};
    double approach_dist_{0.5};
    double heading_rotate_threshold_{0.6};  // rad
    double heading_slowdown_threshold_{0.3}; // rad
    double heading_kp_{2.5};
    double min_turning_linear_vel_{0.05};
    rclcpp::Duration transform_tolerance_{rclcpp::Duration::from_seconds(0.1)};

    // Plan
    nav_msgs::msg::Path global_plan_;
    
    // Debug
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr debug_global_plan_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_lookahead_pub_;
};

}

#endif // DIFF_CONTROLLER__DIFF_CONTROLLER_HPP_