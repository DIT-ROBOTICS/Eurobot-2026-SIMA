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
#include "std_msgs/msg/int16.hpp"
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
        PRE_POSITIONING,
        DELAYING,
        SPRINTING,       // 共用：1~3 號的衝刺，以及 4 號的「第一段前進」
        NAV_HANDOFF_STOP, // 1~3 號專用：衝刺後持續送零速，避免殘留速度進入 Nav2
        SEQ_SPIN,        // 4 號專用：原地向右轉
        SEQ_FORWARD_2,   // 4 號專用：第二段前進
        NAVIGATING,      // 1~3 號專用：Nav2 導航
        STALL_DETECTED,      // <--- 新增：卡死報警
        WAITING_FOR_CANCEL,  // <--- 新增：等待 Nav2 取消
        RETRY_DELAY,
        END
    };

    void startCallback(const std_msgs::msg::String::SharedPtr msg);
    void adjustCallback(const std_msgs::msg::Int16::SharedPtr msg);
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
    double nav_handoff_stop_duration_sec_;
    double first_path_timeout_sec_;
    int first_path_grace_cycles_;
    rclcpp::Time delay_start_time_;
    rclcpp::Time sprint_start_time_;
    rclcpp::Time nav_handoff_start_time_;
    rclcpp::Time first_no_path_feedback_time_;
    bool no_path_feedback_active_ = false;
    int no_path_timeout_cycles_ = 0;
    std::string active_planner_request_;
    std::string active_controller_request_;
    geometry_msgs::msg::PoseStamped active_final_goal_;
    
    // 【新增】4 號機專屬的盲走參數
    double seq_spin_duration_sec_;
    double seq_spin_speed_;
    double seq_forward2_duration_sec_;
    double seq_forward2_speed_;
    rclcpp::Time seq_start_time_;

    // 【新增】排隊調整位置用的變數與參數
    int pre_pos_step_ = 0;
    rclcpp::Time pre_pos_timer_;
    
    // 第一段：前進與旋轉
    double pre_pos_fwd1_sec_;
    double pre_pos_fwd1_speed_;
    double pre_pos_spin1_sec_;
    double pre_pos_spin1_speed_;

    // 第二段：前進與旋轉
    double pre_pos_fwd2_sec_;
    double pre_pos_fwd2_speed_;
    double pre_pos_spin2_sec_;
    double pre_pos_spin2_speed_;

    int consequtive_distance_zero_ = 0;
    float prev_distance_ = 0.0;

    bool is_navigating_ = false;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr adjust_sub_; // 【新增】
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_nav_pub_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planner_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<NavThroughPoses>::SharedPtr nav_client_;
    std::shared_ptr<rclcpp::AsyncParametersClient> vel_smoother_param_client_;
    std::shared_ptr<rclcpp::AsyncParametersClient> global_costmap_param_client_;

    rclcpp::Time retry_start_time_; // 用於重試導航的計時

    // 儲存當前正在執行的 Goal Handle
    rclcpp_action::Client<NavThroughPoses>::GoalHandle::SharedPtr current_goal_handle_;

    std::mutex map_mutex_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace sima_mission

#endif // SIMA_MAIN_SIMA_NAVIGATOR_HPP