#include "special_diff_controller/special_diff_controller.hpp"

#include <algorithm>
#include <cmath>
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

PLUGINLIB_EXPORT_CLASS(special_diff_controller::SpecialDiffController, nav2_core::Controller)

namespace special_diff_controller
{

static double clampAbs(double val, double abs_limit) {
    return max(-abs_limit, min(val, abs_limit));
}

template <typename Iter, typename Getter>
Iter min_by(Iter begin, Iter end, Getter getVal) {
    if (begin == end) return end; 
    auto best = getVal(*begin);
    Iter best_it = begin;
    for (Iter it = std::next(begin); it != end; ++it) {
        auto val = getVal(*it);
        if (val < best) {
            best = val;
            best_it = it;
        }
    }
    return best_it;
}

void SpecialDiffController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent;
    auto node = parent.lock();

    costmap_ros_ = costmap_ros;
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    // Declaration
    declare_parameter_if_not_declared(node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.44));
    declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.4)); // Increased for smoother driving
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(2.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".min_approach_linear_vel", rclcpp::ParameterValue(0.05));
    declare_parameter_if_not_declared(node, plugin_name_ + ".approach_dist", rclcpp::ParameterValue(0.35));
    declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
    
    // State Machine Thresholds (Hysteresis)
    declare_parameter_if_not_declared(node, plugin_name_ + ".heading_error_threshold_to_rotate", rclcpp::ParameterValue(0.15)); // ~8.5 degrees
    declare_parameter_if_not_declared(node, plugin_name_ + ".heading_error_threshold_to_drive", rclcpp::ParameterValue(0.05));  // ~2.8 degrees

    declare_parameter_if_not_declared(node, plugin_name_ + ".heading_kp", rclcpp::ParameterValue(2.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".straight_correction_kp", rclcpp::ParameterValue(0.8));

    declare_parameter_if_not_declared(node, plugin_name_ + ".max_acc_linear", rclcpp::ParameterValue(0.3));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_acc_angular", rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_decel_linear", rclcpp::ParameterValue(1.8));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_decel_linear_emergency", rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_decel_angular", rclcpp::ParameterValue(4.0));

    // Retrieval
    node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
    node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
    node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
    node->get_parameter(plugin_name_ + ".min_approach_linear_vel", min_approach_linear_vel_);
    node->get_parameter(plugin_name_ + ".approach_dist", approach_dist_);
    
    node->get_parameter(plugin_name_ + ".heading_error_threshold_to_rotate", heading_error_threshold_to_rotate_);
    node->get_parameter(plugin_name_ + ".heading_error_threshold_to_drive", heading_error_threshold_to_drive_);
    node->get_parameter(plugin_name_ + ".heading_kp", heading_kp_);
    node->get_parameter(plugin_name_ + ".straight_correction_kp", straight_correction_kp_);

    node->get_parameter(plugin_name_ + ".max_acc_linear", max_acc_linear_);
    node->get_parameter(plugin_name_ + ".max_acc_angular", max_acc_angular_);
    node->get_parameter(plugin_name_ + ".max_decel_linear", max_decel_linear_);
    node->get_parameter(plugin_name_ + ".max_decel_linear_emergency", max_decel_linear_emergency_);
    node->get_parameter(plugin_name_ + ".max_decel_angular", max_decel_angular_);

    double transform_tolerance = 0.1;
    node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

    debug_global_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(
        "/special_diff_controller/debug/global_plan", rclcpp::QoS(1));
    debug_lookahead_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/special_diff_controller/debug/lookahead_point", rclcpp::QoS(1));
        
    RCLCPP_INFO(logger_, "[%s] Configured SpecialDiffController.", plugin_name_.c_str());
}   

void SpecialDiffController::cleanup() {
    debug_global_plan_pub_.reset();
    debug_lookahead_pub_.reset();
}

void SpecialDiffController::activate() {
    debug_global_plan_pub_->on_activate();
    debug_lookahead_pub_->on_activate();
    last_linear_vel_ = 0.0;
    last_angular_vel_ = 0.0;
    current_state_ = DriveState::ROTATING; // Default to rotate first
    last_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
}

void SpecialDiffController::deactivate() {
    debug_global_plan_pub_->on_deactivate();
    debug_lookahead_pub_->on_deactivate();
}

void SpecialDiffController::setPlan(const nav_msgs::msg::Path & path) {   
    global_plan_ = path;

    // 【極端情況優化】：只要收到新路徑（例如避障觸發重新規劃），強制進入旋轉狀態
    // 避免車子用舊的 heading_error 繼續往前盲衝
    // current_state_ = DriveState::ROTATING;

    if (debug_global_plan_pub_->is_activated()) debug_global_plan_pub_->publish(path);
}

void SpecialDiffController::setSpeedLimit(const double & /*speed_limit*/, const bool & /*percentage*/) { return; }

// geometry_msgs::msg::TwistStamped SpecialDiffController::computeVelocityCommands(
//     const geometry_msgs::msg::PoseStamped & pose,
//     const geometry_msgs::msg::Twist & /*velocity*/,
//     nav2_core::GoalChecker * /*goal_checker*/)
// {
//     rclcpp::Time current_time = clock_->now();
//     auto transformed_plan = transformGlobalPlan(pose);

//     // Lookahead logic (Fixed lookahead distance is safer for state machine)
//     auto it = std::find_if(
//         transformed_plan.poses.begin(), transformed_plan.poses.end(),
//         [&](const geometry_msgs::msg::PoseStamped & plan_pose){
//             return hypot(plan_pose.pose.position.x, plan_pose.pose.position.y) >= lookahead_dist_;
//         });
    
//     if (it == transformed_plan.poses.end()){
//         it = std::prev(transformed_plan.poses.end());
//     }

//     const auto & goal_pose = it->pose;

//     if (debug_lookahead_pub_->is_activated()){
//         geometry_msgs::msg::PoseStamped lookahead_msg;
//         lookahead_msg.header = transformed_plan.header;
//         lookahead_msg.pose = goal_pose;
//         debug_lookahead_pub_->publish(lookahead_msg);
//     }

//     // Heading calculation
//     const double x = goal_pose.position.x;
//     const double y = goal_pose.position.y;
//     const double heading_error = std::atan2(y, x);

//     // Obstacle Checking
//     bool is_obstacle = false;
//     std::string costmap_frame = costmap_ros_->getGlobalFrameID();
//     geometry_msgs::msg::PoseStamped goal_pose_local;
//     goal_pose_local.header.frame_id = costmap_ros_->getBaseFrameID();
//     goal_pose_local.header.stamp = current_time;
//     goal_pose_local.pose = goal_pose; 
//     geometry_msgs::msg::PoseStamped goal_pose_global;

//     if (transformPose(tf_, costmap_frame, goal_pose_local, goal_pose_global, transform_tolerance_)) {
//         nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
//         unsigned int mx, my;
//         if (costmap->worldToMap(goal_pose_global.pose.position.x, goal_pose_global.pose.position.y, mx, my)) {
//             unsigned char cost = costmap->getCost(mx, my);
//             if (cost == nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
//                 is_obstacle = true;
//                 RCLCPP_WARN(logger_, "[%s] Obstacle detected! Cost: %d", plugin_name_.c_str(), cost);
//             }
//         }
//     }

//     double target_linear_vel = 0.0;
//     double target_angular_vel = 0.0;

//     // State Machine Hysteresis Update
//     if (current_state_ == DriveState::DRIVING) {
//         if (std::abs(heading_error) > heading_error_threshold_to_rotate_) {
//             current_state_ = DriveState::ROTATING;
//             RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "Switching to ROTATING state. Err: %.2f", heading_error);
//         }
//     } else { // ROTATING
//         if (std::abs(heading_error) < heading_error_threshold_to_drive_) {
//             current_state_ = DriveState::DRIVING;
//             RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "Switching to DRIVING state.");
//         }
//     }

//     // Velocity Generation Based on State
//     if (current_state_ == DriveState::ROTATING) {
//         target_linear_vel = 0.0;
//         // target_angular_vel = heading_kp_ * heading_error;
//         target_angular_vel = std::copysign(1.0, heading_error) * heading_kp_ * std::sqrt(std::abs(heading_error));
//         target_angular_vel = clampAbs(target_angular_vel, max_angular_vel_);
        
//         // Prevent motor deadband when rotating
//         double min_rot_vel = 0.50; 
//         if (std::abs(target_angular_vel) < min_rot_vel) {
//             target_angular_vel = (target_angular_vel > 0) ? min_rot_vel : -min_rot_vel;
//         }
//     } 
//     else { // DRIVING
//         // Only move forward, no curvature. Use tiny P control to maintain straight line.
//         target_angular_vel = straight_correction_kp_ * heading_error;
//         target_angular_vel = clampAbs(target_angular_vel, 0.4); // Hard cap on straight line correction

//         // // Goal approach slowdown
//         // double dist_to_goal = hypot(transformed_plan.poses.back().pose.position.x, 
//         //                             transformed_plan.poses.back().pose.position.y);
        
//         // if (dist_to_goal < approach_dist_) {
//         //     target_linear_vel = std::min(desired_linear_vel_, 
//         //         (dist_to_goal / approach_dist_) * desired_linear_vel_);
//         //     if (dist_to_goal > 0.02) {
//         //         target_linear_vel = std::max(target_linear_vel, min_approach_linear_vel_);
//         //     } else {
//         //         target_linear_vel = 0.0;
//         //     }
//         // } else {
//         //     target_linear_vel = desired_linear_vel_;
//         // }


//         // --- Goal Approach Slowdown (針對全局最終目標) ---
//         double true_dist_to_goal = 10.0; // 預設一個安全的大數字
//         geometry_msgs::msg::PoseStamped robot_pose_in_global;
        
//         // 將機器人當前的 pose 轉換到 global_plan 的座標系來計算絕對距離
//         if (transformPose(tf_, global_plan_.header.frame_id, pose, robot_pose_in_global, transform_tolerance_)) {
//             true_dist_to_goal = euclidean_distance(robot_pose_in_global, global_plan_.poses.back());
//         }

//         // 如果距離最終終點小於設定的減速距離 (例如 0.3m)
//         if (true_dist_to_goal < approach_dist_) {
//             // 依比例線性遞減速度
//             target_linear_vel = std::min(desired_linear_vel_, 
//                 (true_dist_to_goal / approach_dist_) * desired_linear_vel_);
//             target_linear_vel *= 0.7;
//             // 保持一個最小蠕行速度，避免停在半路上，直到距離小於 2 公分才完全煞停
//             // if (true_dist_to_goal > 0.05) {
//             //     target_linear_vel = std::max(target_linear_vel, min_approach_linear_vel_);
//             // } else {
//             //     target_linear_vel = 0.0;
//             // }
            
//             // 加入 Debug 訊息，方便你觀察減速狀態
//             RCLCPP_INFO_THROTTLE(logger_, *clock_, 500, 
//                 "Approaching Final Goal! Dist: %.2f m, Decelerating to Vel: %.2f m/s", 
//                 true_dist_to_goal, target_linear_vel);
//         } else {
//             target_linear_vel = desired_linear_vel_;
//         }
//     }

//     // Emergency Stop
//     if (is_obstacle) {
//         target_linear_vel = 0.0;
//         // Keep rotating if we are just rotating in place, it might clear the path
//         if (current_state_ == DriveState::DRIVING) {
//             target_angular_vel = 0.0; 
//         }
//     }

//     // Acceleration & Deceleration Limiter (Slew Rate)
//     if (last_time_.nanoseconds() != 0) {
//         double dt = (current_time - last_time_).seconds();
//         if (dt > 0.0 && dt < 0.5) {
//             // Linear Limiter
//             double dv_linear = target_linear_vel - last_linear_vel_;
//             bool linear_is_accel = std::abs(target_linear_vel) > std::abs(last_linear_vel_);
//             double decel_limit = is_obstacle ? max_decel_linear_emergency_ : max_decel_linear_; 
//             double linear_limit = linear_is_accel ? max_acc_linear_ : decel_limit;
            
//             dv_linear = std::clamp(dv_linear, -linear_limit * dt, linear_limit * dt);
//             target_linear_vel = last_linear_vel_ + dv_linear;

//             // Angular Limiter
//             double dv_angular = target_angular_vel - last_angular_vel_;
//             bool angular_is_accel = std::abs(target_angular_vel) > std::abs(last_angular_vel_);
//             if (target_angular_vel * last_angular_vel_ < 0) angular_is_accel = false;

//             double angular_limit = angular_is_accel ? max_acc_angular_ : max_decel_angular_;
//             dv_angular = std::clamp(dv_angular, -angular_limit * dt, angular_limit * dt);
//             target_angular_vel = last_angular_vel_ + dv_angular;
//         }
//     }

//     last_linear_vel_ = target_linear_vel;
//     last_angular_vel_ = target_angular_vel;
//     last_time_ = current_time;

//     geometry_msgs::msg::TwistStamped cmd_vel;
//     cmd_vel.header.frame_id = pose.header.frame_id;
//     cmd_vel.header.stamp = current_time;
//     cmd_vel.twist.linear.x = target_linear_vel;
//     cmd_vel.twist.angular.z = target_angular_vel;

//     return cmd_vel;
// }


// 在 configure() 中新增宣告的參數：
// declare_parameter_if_not_declared(node, plugin_name_ + ".system_delay_s", rclcpp::ParameterValue(0.15));
// declare_parameter_if_not_declared(node, plugin_name_ + ".corner_angle_threshold", rclcpp::ParameterValue(0.35)); // 約 20 度
// declare_parameter_if_not_declared(node, plugin_name_ + ".passthrough_tolerance", rclcpp::ParameterValue(0.08)); // 8公分內算抵達



geometry_msgs::msg::TwistStamped SpecialDiffController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/)
{
    rclcpp::Time current_time = clock_->now();
    auto transformed_plan = transformGlobalPlan(pose);

    // ==========================================
    // 0. 共用資源與全局座標
    // ==========================================
    nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> collision_checker(costmap);
    std::vector<geometry_msgs::msg::Point> footprint = costmap_ros_->getRobotFootprint();
    double current_speed = std::abs(velocity.linear.x);

    double robot_global_x = pose.pose.position.x;
    double robot_global_y = pose.pose.position.y;
    double robot_global_yaw = tf2::getYaw(pose.pose.orientation);

    // ==========================================
    // 1. 身心合一：強制看遠，統一目標點
    // ==========================================
    size_t base_idx = 1;
    
    // 【核心修復】：強制忽略距離太近的點，直接消滅「踩在點上」的角度奇異點
    // 讓機器人永遠看著至少 25 公分外的點進行安全偏移
    for (; base_idx < transformed_plan.poses.size() - 1; ++base_idx) {
        if (std::hypot(transformed_plan.poses[base_idx].pose.position.x,
                       transformed_plan.poses[base_idx].pose.position.y) >= 0.25) {
            break;
        }
    }

    auto check_safety = [&](double target_x, double target_y) -> bool {
        double global_t_x = robot_global_x + target_x * std::cos(robot_global_yaw) - target_y * std::sin(robot_global_yaw);
        double global_t_y = robot_global_y + target_x * std::sin(robot_global_yaw) + target_y * std::cos(robot_global_yaw);

        double p_cost = collision_checker.footprintCostAtPose(global_t_x, global_t_y, robot_global_yaw, footprint);
        if (p_cost >= 50.0 || p_cost == nav2_costmap_2d::NO_INFORMATION) return false;

        int num_samples = std::max(5, static_cast<int>(std::hypot(target_x, target_y) / 0.03));
        for (int j = 1; j <= num_samples; ++j) {
            double ratio = j / (double)num_samples;
            double sample_x = robot_global_x + (global_t_x - robot_global_x) * ratio;
            double sample_y = robot_global_y + (global_t_y - robot_global_y) * ratio;
            double cost = collision_checker.footprintCostAtPose(sample_x, sample_y, robot_global_yaw, footprint);
            if (cost >= 128.0 || cost == nav2_costmap_2d::NO_INFORMATION) return false; 
        }
        return true;
    };

    double driving_target_x = transformed_plan.poses[base_idx].pose.position.x;
    double driving_target_y = transformed_plan.poses[base_idx].pose.position.y;
    bool found_target = false;

    // 從遠到近尋找安全腹地
    for (int i = base_idx; i >= 1; --i) {
        double px = transformed_plan.poses[i].pose.position.x;
        double py = transformed_plan.poses[i].pose.position.y;

        if (check_safety(px, py)) {
            driving_target_x = px; driving_target_y = py;
            found_target = true; break;
        }

        if (i > 0) {
            double dx = px - transformed_plan.poses[i-1].pose.position.x;
            double dy = py - transformed_plan.poses[i-1].pose.position.y;
            double len = std::hypot(dx, dy);
            
            if (len > 1e-3) {
                double nx = -dy / len; 
                double ny = dx / len;  

                for (double offset = 0.05; offset <= 0.30; offset += 0.05) {
                    if (check_safety(px + nx * offset, py + ny * offset)) {
                        driving_target_x = px + nx * offset; driving_target_y = py + ny * offset;
                        found_target = true; goto end_search;
                    }
                    if (check_safety(px - nx * offset, py - ny * offset)) {
                        driving_target_x = px - nx * offset; driving_target_y = py - ny * offset;
                        found_target = true; goto end_search;
                    }
                }
            }
        }
    }
    end_search:

    // 【統一目標】：所有的旋轉與直行，都只看這一個點！
    double x = driving_target_x;
    double y = driving_target_y;
    double heading_error = std::atan2(y, x);
    double raw_dist_to_target = std::hypot(x, y);
    
    // 利用目標點的索引來判定是否為最終進場階段
    bool is_final_goal = (base_idx == transformed_plan.poses.size() - 1);

    // ==========================================
    // 2. 專業全車體避障檢測
    // ==========================================
    bool is_obstacle = false;
    double predict_time = 0.5; 
    double check_dist = std::max(0.15, current_speed * predict_time); 

    double projected_x = robot_global_x + check_dist * std::cos(robot_global_yaw);
    double projected_y = robot_global_y + check_dist * std::sin(robot_global_yaw);
    double projected_cost = collision_checker.footprintCostAtPose(projected_x, projected_y, robot_global_yaw, footprint);

    if (projected_cost >= 254.0 || projected_cost == nav2_costmap_2d::NO_INFORMATION) {
        is_obstacle = true; 
    }

    // ==========================================
    // 3. 系統延遲補償
    // ==========================================
    double target_linear_vel = 0.0;
    double target_angular_vel = 0.0;
    double system_delay_s = 0.15; 
    double predicted_dist = std::max(0.0, raw_dist_to_target - (current_speed * system_delay_s));

    // ==========================================
    // 4. 狀態機切換與目標穿透判斷
    // ==========================================
    double passthrough_tolerance = 0.08; 
    bool passed_target = false;

    if (is_final_goal) {
        passed_target = (x < -0.02); // 終點必須完美對位，稍微衝過頭才切換
    } else {
        // 中繼點容錯率大，只要靠近或越過就切換
        passed_target = (raw_dist_to_target < passthrough_tolerance) || (x < 0.0 && raw_dist_to_target < 0.2);
    }

    if (current_state_ == DriveState::DRIVING) {
        if (std::abs(heading_error) > heading_error_threshold_to_rotate_ || passed_target) {
            current_state_ = DriveState::ROTATING;
        }
    } else { // ROTATING
        if (std::abs(heading_error) < heading_error_threshold_to_drive_) {
            current_state_ = DriveState::DRIVING;
        }
    }

    // ==========================================
    // 5. 速度指令生成
    // ==========================================
    if (current_state_ == DriveState::ROTATING) {
        target_linear_vel = 0.0;
        target_angular_vel = std::copysign(1.0, heading_error) * heading_kp_ * std::sqrt(std::abs(heading_error));
        target_angular_vel = clampAbs(target_angular_vel, max_angular_vel_);
        
        double min_rot_vel_ = 0.25; 
        if (std::abs(target_angular_vel) < min_rot_vel_) {
            target_angular_vel = (target_angular_vel > 0) ? min_rot_vel_ : -min_rot_vel_;
        }
    } else { // DRIVING
        target_angular_vel = straight_correction_kp_ * heading_error;
        target_angular_vel = clampAbs(target_angular_vel, 0.4);

        if (predicted_dist < approach_dist_) {
            target_linear_vel = std::min(desired_linear_vel_, (predicted_dist / approach_dist_) * desired_linear_vel_);
            
            // 如果是終點，保留最小速度嚕進去；如果是中繼點，直接全速衝過
            double stop_distance = is_final_goal ? 0.01 : 0.0;
            
            if (predicted_dist > stop_distance) {
                target_linear_vel = std::max(target_linear_vel, min_approach_linear_vel_);
            } else {
                target_linear_vel = 0.0; 
            }
        } else {
            target_linear_vel = desired_linear_vel_;
        }
    }

    if (is_obstacle) {
        target_linear_vel = 0.0;
    }

    // ==========================================
    // 6. Slew Rate 加減速限制
    // ==========================================
    if (last_time_.nanoseconds() != 0) {
        double dt = (current_time - last_time_).seconds();
        if (dt > 0.0 && dt < 0.5) {
            double dv_linear = target_linear_vel - last_linear_vel_;
            bool linear_is_accel = std::abs(target_linear_vel) > std::abs(last_linear_vel_);
            double decel_limit = is_obstacle ? max_decel_linear_emergency_ : max_decel_linear_; 
            double linear_limit = linear_is_accel ? max_acc_linear_ : decel_limit;
            
            dv_linear = std::clamp(dv_linear, -linear_limit * dt, linear_limit * dt);
            target_linear_vel = last_linear_vel_ + dv_linear;

            double dv_angular = target_angular_vel - last_angular_vel_;
            bool angular_is_accel = std::abs(target_angular_vel) > std::abs(last_angular_vel_);
            if (target_angular_vel * last_angular_vel_ < 0) angular_is_accel = false;

            double angular_limit = angular_is_accel ? max_acc_angular_ : max_decel_angular_;
            dv_angular = std::clamp(dv_angular, -angular_limit * dt, angular_limit * dt);
            target_angular_vel = last_angular_vel_ + dv_angular;
        }
    }

    last_linear_vel_ = target_linear_vel;
    last_angular_vel_ = target_angular_vel;
    last_time_ = current_time;

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = current_time;
    cmd_vel.twist.linear.x = target_linear_vel;
    cmd_vel.twist.angular.z = target_angular_vel;

    // ==========================================
    // 7. 乾淨版 X光透視 Debug 儀表板
    // ==========================================
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000,
        "\n--- [Unified DiffController X-Ray] ---\n"
        "1. State      : %s\n"
        "2. Target(x,y): (%.3f, %.3f) | Offset: %s | Final: %s\n"
        "3. Distances  : Raw=%.3f, Predicted=%.3f\n"
        "4. HeadingErr : %.3f rad\n"
        "5. Final Cmd  : Linear=%.3f, Angular=%.3f\n"
        "--------------------------------------",
        current_state_ == DriveState::DRIVING ? "DRIVING ⬆️" : "ROTATING 🔄",
        driving_target_x, driving_target_y, found_target ? "YES" : "NO", is_final_goal ? "YES" : "NO",
        raw_dist_to_target, predicted_dist,
        heading_error,
        target_linear_vel, target_angular_vel);

    return cmd_vel;
}




// ... [Keep transformGlobalPlan and transformPose exactly as they were in your original code] ...
// (為節省版面，這兩個標準的坐標系轉換函式與你原始檔案中一致，請直接從舊版複製貼上即可)

nav_msgs::msg::Path SpecialDiffController::transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose) {
    if (global_plan_.poses.empty()) throw nav2_core::PlannerException("SpecialDiffController: received plan with zero length");
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!transformPose(tf_, global_plan_.header.frame_id, pose, robot_pose, transform_tolerance_))
        throw nav2_core::PlannerException("SpecialDiffController: Could not transform robot pose");

    nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
    const double dist_threshold = max(costmap->getSizeInMetersX(), costmap->getSizeInMetersY()) / 2.0;
    
    auto transformation_begin = min_by(global_plan_.poses.begin(), global_plan_.poses.end(),
        [&robot_pose](const geometry_msgs::msg::PoseStamped & plan_pose){
            return euclidean_distance(robot_pose, plan_pose);
        });
    
    auto transformation_end = std::find_if(transformation_begin, global_plan_.poses.end(),
        [&](const geometry_msgs::msg::PoseStamped & plan_pose){
            return euclidean_distance(robot_pose, plan_pose) > dist_threshold;
        });
    
    auto toLocal = [&](const geometry_msgs::msg::PoseStamped & ps) {
        geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
        stamped_pose.header.frame_id = global_plan_.header.frame_id;
        stamped_pose.header.stamp = pose.header.stamp;
        stamped_pose.pose = ps.pose;
        (void)transformPose(tf_, costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose, transform_tolerance_);
        return transformed_pose;
    };

    nav_msgs::msg::Path transformed;
    std::transform(transformation_begin, transformation_end, std::back_inserter(transformed.poses), toLocal);
    transformed.header.frame_id = costmap_ros_->getBaseFrameID();
    transformed.header.stamp = pose.header.stamp;
    global_plan_.poses.erase(global_plan_.poses.begin(), transformation_begin);

    if (debug_global_plan_pub_->is_activated()) debug_global_plan_pub_->publish(transformed);
    if (transformed.poses.empty()) throw nav2_core::PlannerException("SpecialDiffController: transformed plan has 0 poses");
    return transformed;
}

bool SpecialDiffController::transformPose(
    const std::shared_ptr<tf2_ros::Buffer> tf, const std::string & frame,
    const geometry_msgs::msg::PoseStamped & in_pose, geometry_msgs::msg::PoseStamped & out_pose,
    const rclcpp::Duration & transform_tolerance) const 
{
    if (in_pose.header.frame_id == frame) { out_pose = in_pose; return true; }
    try {
        tf->transform(in_pose, out_pose, frame); return true;
    } catch (tf2::ExtrapolationException &) {
        auto transform = tf->lookupTransform(frame, in_pose.header.frame_id, tf2::TimePointZero);
        if ((rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) > transform_tolerance) {
            return false;
        }
        tf2::doTransform(in_pose, out_pose, transform); return true;
    } catch (tf2::TransformException &) {
        return false;
    }
}

}  // namespace special_diff_controller