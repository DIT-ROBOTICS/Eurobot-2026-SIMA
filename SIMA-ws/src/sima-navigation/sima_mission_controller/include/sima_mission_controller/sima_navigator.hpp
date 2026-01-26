#ifndef SIMA_NAVIGATOR_HPP_
#define SIMA_NAVIGATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <vector>
#include <mutex>
#include <optional>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace sima_mission
{

class SimaNavigator : public rclcpp::Node
{
public:
    using NavThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavThroughPoses>;

    SimaNavigator();

private:
    // Callbacks
    void startCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    // Logic
    void executeMission();
    std::optional<geometry_msgs::msg::PoseStamped> findNearestSafePoint(double wx, double wy, double search_r_m = 0.8);
    void worldToMap(double wx, double wy, int& mx, int& my);
    void mapToWorld(int mx, int my, double& wx, double& wy);
    std::vector<std::pair<double, double>> parseWaypoints(const std::vector<double>& flat_points);
    
    // Action Client Callbacks
    void goalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle);
    void feedbackCallback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavThroughPoses::Feedback> feedback);
    void resultCallback(const GoalHandleNav::WrappedResult & result);

    // Variables
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_pub_;
    rclcpp_action::Client<NavThroughPoses>::SharedPtr nav_client_;

    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
    std::mutex map_mutex_;
    bool is_navigating_ = false;
    bool last_start_signal_ = false;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace sima_mission

#endif // SIMA_NAVIGATOR_HPP_