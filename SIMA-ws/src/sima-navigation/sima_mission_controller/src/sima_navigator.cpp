#include "sima_mission_controller/sima_navigator.hpp"
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

namespace sima_mission
{

SimaNavigator::SimaNavigator() : Node("sima_navigator")
{
    // 1. Initialize Subscriptions and Publications
    // Trigger topic: "ros2 topic pub /start_sima std_msgs/msg/Bool '{data: true}' -1"
    start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/start_sima", 10, std::bind(&SimaNavigator::startCallback, this, std::placeholders::_1));

    // Subscribe to Global Costmap
    rclcpp::QoS map_qos(1);
    map_qos.transient_local();
    map_qos.reliable();
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap", map_qos, std::bind(&SimaNavigator::costmapCallback, this, std::placeholders::_1));

    // Publish Controller switch signal
    controller_pub_ = this->create_publisher<std_msgs::msg::String>("/controller_type_thru", map_qos);

    // Nav2 Action Client
    nav_client_ = rclcpp_action::create_client<NavThroughPoses>(this, "navigate_through_poses");

    RCLCPP_INFO(this->get_logger(), "=== SIMA Navigator Ready. Waiting for /start_sima ===");
}

void SimaNavigator::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    latest_costmap_ = msg;
}

void SimaNavigator::startCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data && !is_navigating_) {
        RCLCPP_INFO(this->get_logger(), "Received START signal!");
        executeMission();
    } else {
        RCLCPP_WARN(this->get_logger(), "Ignore start signal (already running or false)");
    }
}

void SimaNavigator::executeMission()
{
    is_navigating_ = true;

    // 1. Switch Controller
    auto ctrl_msg = std_msgs::msg::String();
    ctrl_msg.data = "Diff";
    controller_pub_->publish(ctrl_msg);
    // Publish multiple times to ensure reception (simple but effective)
    rclcpp::sleep_for(100ms);
    controller_pub_->publish(ctrl_msg);
    RCLCPP_INFO(this->get_logger(), "Controller switched to Diff");

    // 2. Check if costmap exists
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!latest_costmap_) {
            RCLCPP_ERROR(this->get_logger(), "No Global Costmap received yet! Aborting.");
            is_navigating_ = false;
            return;
        }
    }

    // 3. Define Waypoints (hardcoded or read from Parameter)       ->    TODO: change to parameter
    std::vector<std::pair<double, double>> raw_points = {
        {1.5, 0.5},
        {2.0, 0.7},
        {2.39, 0.5}
    };

    // 4. Safety Check and Path Generation
    auto goal_msg = NavThroughPoses::Goal();
    goal_msg.poses.clear();

    RCLCPP_INFO(this->get_logger(), "Checking waypoints against costmap...");

    for (const auto& pt : raw_points) {
        // Use spiral search to find a safe point
        auto safe_pose_opt = findNearestSafePoint(pt.first, pt.second);

        if (safe_pose_opt.has_value()) {
            goal_msg.poses.push_back(safe_pose_opt.value());
        } else {
            RCLCPP_WARN(this->get_logger(), "Skipping waypoint (%.2f, %.2f) because no safe point was found nearby!", pt.first, pt.second);
        }
    }

    if (goal_msg.poses.empty()) {
        // Increase searching radius for final waypoint(goal point)
        auto goal_pose = findNearestSafePoint(raw_points.back().first, raw_points.back().second, 1.5);
        goal_msg.poses.push_back(goal_pose.value());
    }

    // Debug: List final waypoints
    RCLCPP_INFO(this->get_logger(), "Final waypoints sent to Nav2:");
    for (const auto& pose : goal_msg.poses) {
        RCLCPP_INFO(this->get_logger(), " -> (%.2f, %.2f)", pose.pose.position.x, pose.pose.position.y);
    }

    // 5. Wait for Action Server
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Nav2 Action Server not available!");
        is_navigating_ = false;
        return;
    }

    // 6. Send waypoints to Nav2
    RCLCPP_INFO(this->get_logger(), "Sending safe waypoints to Nav2...");
    
    auto send_goal_options = rclcpp_action::Client<NavThroughPoses>::SendGoalOptions();
    send_goal_options.goal_response_callback = 
        std::bind(&SimaNavigator::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = 
        std::bind(&SimaNavigator::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = 
        std::bind(&SimaNavigator::resultCallback, this, std::placeholders::_1);

    nav_client_->async_send_goal(goal_msg, send_goal_options);
}

std::optional<geometry_msgs::msg::PoseStamped> SimaNavigator::findNearestSafePoint(double wx, double wy, double search_r_m)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.orientation.w = 1.0;

    std::lock_guard<std::mutex> lock(map_mutex_);
    
    int mx, my;
    worldToMap(wx, wy, mx, my);

    int width = latest_costmap_->info.width;
    int height = latest_costmap_->info.height;
    
    // Check waypoint Cost
    int index = my * width + mx;
    int8_t cost = latest_costmap_->data[index];

    // If safe (Cost < 50 and not -1/unknown)
    if (cost >= 0 && cost < 50) {
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        return pose;
    }

    RCLCPP_WARN(this->get_logger(), "Point (%.2f, %.2f) is unsafe (Cost: %d). Searching nearby...", wx, wy, cost);

    // Spiral Search for nearest safe point
    int search_radius_cells = static_cast<int>(search_r_m / latest_costmap_->info.resolution);

    for (int r = 1; r < search_radius_cells; ++r) {
        for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                // Only check the perimeter of the square ring
                if (std::abs(dx) != r && std::abs(dy) != r) continue;

                int check_x = mx + dx;
                int check_y = my + dy;

                if (check_x >= 0 && check_x < width && check_y >= 0 && check_y < height) {
                    int idx = check_y * width + check_x;
                    int8_t c = latest_costmap_->data[idx];
                    
                    // Found absolutely safe point (Cost == 0)
                    if (c == 0) {
                        double safe_wx, safe_wy;
                        mapToWorld(check_x, check_y, safe_wx, safe_wy);
                        pose.pose.position.x = safe_wx;
                        pose.pose.position.y = safe_wy;
                        RCLCPP_INFO(this->get_logger(), " -> Found safe point at (%.2f, %.2f)", safe_wx, safe_wy);
                        return pose;
                    }
                }
            }
        }
    }

    // If not found, then delete waypoiint
    RCLCPP_ERROR(this->get_logger(), "Could not find safe point nearby (%.2f, %.2f) within range %.2fm!", wx, wy, search_r_m);
    return std::nullopt; 
}

void SimaNavigator::worldToMap(double wx, double wy, int& mx, int& my)
{
    double origin_x = latest_costmap_->info.origin.position.x;
    double origin_y = latest_costmap_->info.origin.position.y;
    double res = latest_costmap_->info.resolution;
    mx = static_cast<int>((wx - origin_x) / res);
    my = static_cast<int>((wy - origin_y) / res);
}

void SimaNavigator::mapToWorld(int mx, int my, double& wx, double& wy)
{
    double origin_x = latest_costmap_->info.origin.position.x;
    double origin_y = latest_costmap_->info.origin.position.y;
    double res = latest_costmap_->info.resolution;
    wx = (mx * res) + origin_x + (res / 2.0);
    wy = (my * res) + origin_y + (res / 2.0);
}

void SimaNavigator::goalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        is_navigating_ = false;
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void SimaNavigator::feedbackCallback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavThroughPoses::Feedback> feedback)
{
    // Advanced monitoring can be done here (e.g., check if the path ahead is blocked every few seconds)
    // Currently, just print the remaining distance
    static int log_counter = 0;
    if (log_counter++ % 20 == 0) { // Reduce log frequency
        RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f", feedback->distance_remaining);
    }
}

void SimaNavigator::resultCallback(const GoalHandleNav::WrappedResult & result)
{
    is_navigating_ = false;
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Mission Completed Successfully!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Mission Aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Mission Canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
    }
}

} // namespace sima_mission

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sima_mission::SimaNavigator>());
    rclcpp::shutdown();
    return 0;
}