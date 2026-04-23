// #ifndef SIMA_NAVIGATOR_HPP_
// #define SIMA_NAVIGATOR_HPP_

// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "nav2_msgs/action/navigate_through_poses.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "std_msgs/msg/int16.hpp"
// #include <vector>
// #include <mutex>
// #include <optional>

// #include "tf2_ros/buffer.h"
// #include "tf2_ros/transform_listener.h"

// namespace sima_mission
// {

// class SimaNavigator : public rclcpp::Node
// {
// public:
//     enum class State
//     {
//         IDLE,
//         SPRINTING,
//         NAVIGATING,
//     };
// public:
//     using NavThroughPoses = nav2_msgs::action::NavigateThroughPoses;
//     using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavThroughPoses>;

//     SimaNavigator();

// private:
//     // Callbacks
//     void startCallback(const std_msgs::msg::Int16::SharedPtr msg);
//     void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

//     // Logic
//     void executeMission();
//     std::optional<geometry_msgs::msg::PoseStamped> findNearestSafePoint(double wx, double wy, double search_r_m = 0.8);
//     void worldToMap(double wx, double wy, int& mx, int& my);
//     void mapToWorld(int mx, int my, double& wx, double& wy);
//     std::vector<std::pair<double, double>> parseWaypoints(const std::vector<double>& flat_points);
//     void controlLoop();
//     void stopRobot();

//     // Action Client Callbacks
//     void goalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle);
//     void feedbackCallback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavThroughPoses::Feedback> feedback);
//     void resultCallback(const GoalHandleNav::WrappedResult & result);

//     // Variables
//     rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr start_sub_;
//     rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_pub_;
//     rclcpp_action::Client<NavThroughPoses>::SharedPtr nav_client_;

//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
//     nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
//     std::mutex map_mutex_;
//     bool is_navigating_ = false;
//     int last_start_signal_ = 0;

//     State current_state_ = State::IDLE;
//     rclcpp::Time sprint_start_time_;
//     double sprint_duration_sec_ = 1.0; // Duration to sprint before switching to navigation
//     double sprint_speed_ = 0.5; // Speed to use during sprinting (m/s)

//     std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
// };

// } // namespace sima_mission
// #endif // SIMA_NAVIGATOR_HPP_









// #ifndef SIMA_NAVIGATOR_HPP_
// #define SIMA_NAVIGATOR_HPP_

// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "nav2_msgs/action/navigate_through_poses.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "std_msgs/msg/int16.hpp"
// #include <vector>
// #include <mutex>
// #include <optional>

// #include "tf2_ros/buffer.h"
// #include "tf2_ros/transform_listener.h"

// namespace sima_mission
// {

// enum class MissionType {
//     PEACE = 0,
//     AGGRESSIVE = 1
// };

// class SimaNavigator : public rclcpp::Node
// {
// public:
//     enum class State
//     {
//         IDLE,
//         DELAYING,
//         SPRINTING,
//         NAVIGATING,
//         END,
//     };
// public:
//     using NavThroughPoses = nav2_msgs::action::NavigateThroughPoses;
//     using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavThroughPoses>;

//     SimaNavigator();

// private:
//     // Callbacks
//     void startCallback(const std_msgs::msg::String::SharedPtr msg);
//     void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

//     // Logic
//     void executeMission();
//     std::optional<geometry_msgs::msg::PoseStamped> findNearestSafePoint(double wx, double wy, double search_r_m = 0.8);
//     void worldToMap(double wx, double wy, int& mx, int& my);
//     void mapToWorld(int mx, int my, double& wx, double& wy);
//     std::vector<std::pair<double, double>> parseWaypoints(const std::vector<double>& flat_points);
//     void controlLoop();
//     void stopRobot();

//     // Action Client Callbacks
//     void goalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle);
//     void feedbackCallback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavThroughPoses::Feedback> feedback);
//     void resultCallback(const GoalHandleNav::WrappedResult & result);

//     // Variables
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_sub_;
//     rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_pub_;
//     rclcpp_action::Client<NavThroughPoses>::SharedPtr nav_client_;

//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

//     nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
//     std::mutex map_mutex_;
//     bool is_navigating_ = false;
//     int last_start_signal_ = 0;
//     std::string target_pantry_;

//     State current_state_ = State::IDLE;
//     rclcpp::Time sprint_start_time_;
//     rclcpp::Time delay_start_time_;
//     double sprint_duration_sec_ = 1.0; // Duration to sprint before switching to navigation
//     double sprint_speed_ = 0.5; // Speed to use during sprinting (m/s)
//     int sima_id_;

//     std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
// };

// } // namespace sima_mission

// #endif // SIMA_NAVIGATOR_HPP_




// #ifndef SIMA_MAIN_SIMA_NAVIGATOR_HPP
// #define SIMA_MAIN_SIMA_NAVIGATOR_HPP

// #include <memory>
// #include <string>
// #include <vector>
// #include <chrono>
// #include <mutex>
// #include <optional>
// #include <exception>

// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "rclcpp/parameter_client.hpp" // 新增：用於動態調整 velocity_smoother

// #include "std_msgs/msg/string.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"
// #include "nav2_msgs/action/navigate_through_poses.hpp"

// #include "tf2_ros/buffer.h"
// #include "tf2_ros/transform_listener.h"

// namespace sima_mission
// {

// // 定義導航策略類型
// enum class MissionType {
//     PEACE = 1,
//     NORMAL = 2,
//     AGGRESSIVE = 3
// };

// enum class PantryStatus {
//     UNKNOWN = -1,
//     EMPTY = 0,
//     OURS = 1,
//     ENEMY = 2,
//     EVEN = 3,
//     CANATTACK = 4
// };

// class SimaNavigator : public rclcpp::Node
// {
// public:
//     using NavThroughPoses = nav2_msgs::action::NavigateThroughPoses;
//     using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavThroughPoses>;

//     SimaNavigator();

// private:
//     enum class State {
//         IDLE,
//         DELAYING,
//         SPRINTING,
//         NAVIGATING,
//         END
//     };

//     // Callbacks
//     void startCallback(const std_msgs::msg::String::SharedPtr msg);
//     void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
//     void controlLoop();

//     // Mission Execution
//     void stopRobot();
//     void executeMission();

//     // Helper functions
//     std::vector<std::pair<double, double>> parseWaypoints(const std::vector<double>& flat_points);
//     std::optional<geometry_msgs::msg::PoseStamped> findNearestSafePoint(double wx, double wy, double search_r_m = 0.5);
//     void worldToMap(double wx, double wy, int& mx, int& my);
//     void mapToWorld(int mx, int my, double& wx, double& wy);

//     // Action Client Callbacks
//     void goalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle);
//     void feedbackCallback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavThroughPoses::Feedback> feedback);
//     void resultCallback(const GoalHandleNav::WrappedResult & result);

//     // Node state and config
//     State current_state_ = State::IDLE;
//     MissionType current_mission_ = MissionType::AGGRESSIVE;
//     PantryStatus target_status_ = PantryStatus::UNKNOWN;

//     int sima_id_;
//     std::string target_pantry_;
//     double sprint_duration_sec_;
//     double sprint_speed_;
//     rclcpp::Time delay_start_time_;
//     rclcpp::Time sprint_start_time_;
//     bool is_navigating_ = false;

//     // ROS 2 Interfaces
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_sub_;
//     rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
//     // 用於切換 Behavior Tree 裡面的 Planner 與 Controller
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_pub_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planner_pub_;

//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp_action::Client<NavThroughPoses>::SharedPtr nav_client_;

//     // 新增：Velocity Smoother 參數客戶端
//     std::shared_ptr<rclcpp::AsyncParametersClient> vel_smoother_param_client_;
//     std::shared_ptr<rclcpp::AsyncParametersClient> global_costmap_param_client_;

//     // Map handling
//     std::mutex map_mutex_;
//     nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;

//     // TF
//     std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
//     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
// };

// } // namespace sima_mission

// #endif // SIMA_MAIN_SIMA_NAVIGATOR_HPP


#ifndef SIMA_MAIN_SIMA_NAVIGATOR_HPP
#define SIMA_MAIN_SIMA_NAVIGATOR_HPP

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <mutex>
#include <optional>
#include <exception>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/parameter_client.hpp" 

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace sima_mission
{

enum class MissionType {
    PEACE = 1,
    NORMAL = 2,
    AGGRESSIVE = 3
};

enum class PantryStatus {
    UNKNOWN = -1,
    EMPTY = 0,
    OURS = 1,
    ENEMY = 2,
    EVEN = 3,
    CANATTACK = 4
};

class SimaNavigator : public rclcpp::Node
{
public:
    using NavThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavThroughPoses>;

    SimaNavigator();

private:
    // 【修改】擴增狀態機，加入 4 號機專用的盲走狀態
    enum class State {
        IDLE,
        DELAYING,
        SPRINTING,       // 共用：1~3 號的衝刺，以及 4 號的「第一段前進」
        SEQ_SPIN,        // 4 號專用：原地向右轉
        SEQ_FORWARD_2,   // 4 號專用：第二段前進
        NAVIGATING,      // 1~3 號專用：Nav2 導航
        END
    };

    void startCallback(const std_msgs::msg::String::SharedPtr msg);
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void controlLoop();

    void stopRobot();
    void executeMission();
    double calculateVelocityRatio(double elapsed_sec, double total_sec);

    std::vector<std::pair<double, double>> parseWaypoints(const std::vector<double>& flat_points);
    std::optional<geometry_msgs::msg::PoseStamped> findNearestSafePoint(double wx, double wy, double search_r_m = 0.5);
    void worldToMap(double wx, double wy, int& mx, int& my);
    void mapToWorld(int mx, int my, double& wx, double& wy);

    void goalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle);
    void feedbackCallback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavThroughPoses::Feedback> feedback);
    void resultCallback(const GoalHandleNav::WrappedResult & result);

    State current_state_ = State::IDLE;
    MissionType current_mission_ = MissionType::AGGRESSIVE;
    PantryStatus target_status_ = PantryStatus::UNKNOWN;

    int sima_id_;
    std::string target_pantry_;
    
    // 時間與速度參數
    double sprint_duration_sec_;
    double sprint_speed_;
    rclcpp::Time delay_start_time_;
    rclcpp::Time sprint_start_time_;
    
    // 【新增】4 號機專屬的盲走參數
    double seq_spin_duration_sec_;
    double seq_spin_speed_;
    double seq_forward2_duration_sec_;
    double seq_forward2_speed_;
    rclcpp::Time seq_start_time_;

    bool is_navigating_ = false;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planner_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<NavThroughPoses>::SharedPtr nav_client_;
    std::shared_ptr<rclcpp::AsyncParametersClient> vel_smoother_param_client_;
    std::shared_ptr<rclcpp::AsyncParametersClient> global_costmap_param_client_;

    std::mutex map_mutex_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace sima_mission

#endif // SIMA_MAIN_SIMA_NAVIGATOR_HPP