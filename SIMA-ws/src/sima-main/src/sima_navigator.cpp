#include "sima-main/sima_navigator.hpp"
#include <cmath>
#include <sstream>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace sima_mission
{

SimaNavigator::SimaNavigator() : Node("sima_navigator")
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->declare_parameter("start_delay_seconds", 0.0);
    this->declare_parameter("sprint_duration_sec", 1.0);
    this->declare_parameter("sprint_speed", 0.5);
    this->declare_parameter("nav_handoff_stop_duration_sec", 0.4);
    this->declare_parameter("first_path_timeout_sec", 3.0);
    this->declare_parameter("first_path_grace_cycles", 2);
    this->declare_parameter("enable_dynamic_costmap_inflation", false);
    this->declare_parameter("sima_id", 1);

    // 【新增】宣告 4 號機的特種盲走參數
    this->declare_parameter("seq_spin_duration_sec", 1.0);
    this->declare_parameter("seq_spin_speed", -1.5); // 預設負數為向右轉
    this->declare_parameter("seq_forward2_duration_sec", 1.5);
    this->declare_parameter("seq_forward2_speed", 0.4);

    // 第一段參數
    this->declare_parameter("pre_pos_fwd1_sec", 0.5);
    this->declare_parameter("pre_pos_fwd1_speed", 0.3);
    this->declare_parameter("pre_pos_spin1_sec", 0.5);
    this->declare_parameter("pre_pos_spin1_speed", 0.5);
    
    // 第二段參數
    this->declare_parameter("pre_pos_fwd2_sec", 0.5);
    this->declare_parameter("pre_pos_fwd2_speed", 0.3);
    this->declare_parameter("pre_pos_spin2_sec", 0.5);
    this->declare_parameter("pre_pos_spin2_speed", 0.5);

    sprint_duration_sec_ = this->get_parameter("sprint_duration_sec").as_double();
    sprint_speed_ = this->get_parameter("sprint_speed").as_double();
    nav_handoff_stop_duration_sec_ = this->get_parameter("nav_handoff_stop_duration_sec").as_double();
    first_path_timeout_sec_ = this->get_parameter("first_path_timeout_sec").as_double();
    first_path_grace_cycles_ = this->get_parameter("first_path_grace_cycles").as_int();
    sima_id_ = this->get_parameter("sima_id").as_int();

    seq_spin_duration_sec_ = this->get_parameter("seq_spin_duration_sec").as_double();
    seq_spin_speed_ = this->get_parameter("seq_spin_speed").as_double();
    seq_forward2_duration_sec_ = this->get_parameter("seq_forward2_duration_sec").as_double();
    seq_forward2_speed_ = this->get_parameter("seq_forward2_speed").as_double();

    // 讀取第一段參數
    pre_pos_fwd1_sec_ = this->get_parameter("pre_pos_fwd1_sec").as_double();
    pre_pos_fwd1_speed_ = this->get_parameter("pre_pos_fwd1_speed").as_double();
    pre_pos_spin1_sec_ = this->get_parameter("pre_pos_spin1_sec").as_double();
    pre_pos_spin1_speed_ = this->get_parameter("pre_pos_spin1_speed").as_double();

    // 讀取第二段參數
    pre_pos_fwd2_sec_ = this->get_parameter("pre_pos_fwd2_sec").as_double();
    pre_pos_fwd2_speed_ = this->get_parameter("pre_pos_fwd2_speed").as_double();
    pre_pos_spin2_sec_ = this->get_parameter("pre_pos_spin2_sec").as_double();
    pre_pos_spin2_speed_ = this->get_parameter("pre_pos_spin2_speed").as_double();

    // QoS for state keeping (Transient Local 讓晚來的訂閱者也能收到最新狀態)
    rclcpp::QoS state_qos(1);
    state_qos.transient_local();
    state_qos.reliable();

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    cmd_vel_nav_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_nav", 10);
    controller_pub_ = this->create_publisher<std_msgs::msg::String>("/controller_type_thru", state_qos);
    planner_pub_ = this->create_publisher<std_msgs::msg::String>("/planner_type_thru", state_qos);

    std::string start_topic = "/sima_" + std::to_string(sima_id_) + "/goal";
    start_sub_ = this->create_subscription<std_msgs::msg::String>(
        start_topic, 10, std::bind(&SimaNavigator::startCallback, this, std::placeholders::_1));
    
    std::string adjust_topic = "/sima_" + std::to_string(sima_id_) + "/adjust";
    adjust_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        adjust_topic, 10, std::bind(&SimaNavigator::adjustCallback, this, std::placeholders::_1));
    
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap", state_qos, std::bind(&SimaNavigator::costmapCallback, this, std::placeholders::_1));

    nav_client_ = rclcpp_action::create_client<NavThroughPoses>(this, "navigate_through_poses");
    vel_smoother_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "velocity_smoother");
    global_costmap_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "global_costmap/global_costmap");

    timer_ = this->create_wall_timer(20ms, std::bind(&SimaNavigator::controlLoop, this));

    current_state_ = State::IDLE;

    RCLCPP_INFO(this->get_logger(), "=== SIMA %d Navigator Ready. Waiting for target on topic: %s ===", sima_id_, start_topic.c_str());
    if (sima_id_ == 4 || sima_id_ == 14) {
        RCLCPP_INFO(this->get_logger(), "Notice: SIMA 4 or 14 will execute fixed OPEN-LOOP sequence instead of Nav2.");
    }
}

// ... costmapCallback 與 startCallback 保持完全不變 ...
void SimaNavigator::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    latest_costmap_ = msg;
}

void SimaNavigator::adjustCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
    // 只有在 IDLE 狀態下才能進入調整狀態，避免干擾正常任務
    if (current_state_ == State::IDLE && msg->data == 1) {
        RCLCPP_INFO(this->get_logger(), "Received Adjust Signal! Starting Pre-Positioning.");
        current_state_ = State::PRE_POSITIONING;
        pre_pos_step_ = 0;
        pre_pos_timer_ = this->now();
    }
}

void SimaNavigator::startCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if ((current_state_ == State::IDLE || current_state_ == State::PRE_POSITIONING) && !msg->data.empty()) {
        std::string raw_data = msg->data;

        if (current_state_ == State::PRE_POSITIONING) {
            RCLCPP_WARN(this->get_logger(), "Pre-Positioning INTERRUPTED by formal Start Signal!");
            stopRobot();
        }
        
        std::stringstream ss(raw_data);
        std::string item;
        std::vector<std::string> tokens;
        while (std::getline(ss, item, '|')) {
            tokens.push_back(item);
        }

        if (tokens.size() >= 3) {
            target_pantry_ = tokens[0];
            try {
                int mission_val = std::stoi(tokens[1]);
                if (mission_val == 1) current_mission_ = MissionType::PEACE;
                else if (mission_val == 2) current_mission_ = MissionType::NORMAL;
                else current_mission_ = MissionType::AGGRESSIVE;
                
                target_status_ = static_cast<PantryStatus>(std::stoi(tokens[2]));
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "Parse error. Defaulting to PEACE/UNKNOWN.");
                current_mission_ = MissionType::PEACE;
                target_status_ = PantryStatus::UNKNOWN;
            }
        } else if (tokens.size() == 2) {
            target_pantry_ = tokens[0];
            current_mission_ = (std::stoi(tokens[1]) == 3) ? MissionType::AGGRESSIVE : MissionType::PEACE;
            target_status_ = PantryStatus::UNKNOWN;
        } else {
            target_pantry_ = raw_data;
            current_mission_ = MissionType::PEACE;
            target_status_ = PantryStatus::UNKNOWN;
        }

        RCLCPP_INFO(this->get_logger(), "Parsed - Target: %s, Mission: %d, Status: %d", 
                    target_pantry_.c_str(), static_cast<int>(current_mission_), static_cast<int>(target_status_));
        
        double start_delay = this->get_parameter("start_delay_seconds").as_double();
        
        if (start_delay > 0.0) {
            RCLCPP_INFO(this->get_logger(), "Entering delay phase for %.1f seconds...", start_delay);
            current_state_ = State::DELAYING;
            delay_start_time_ = this->now();
        } else {
            RCLCPP_INFO(this->get_logger(), "Starting sprinting phase...");
            current_state_ = State::SPRINTING;
            sprint_start_time_ = this->now();
        }
    }
}

// 【核心修改】擴展控制迴圈，加入 4 號的盲走狀態機
void SimaNavigator::controlLoop()
{
    if (current_state_ == State::IDLE) {
        // stopRobot();
    }
    if (current_state_ == State::PRE_POSITIONING) {
        int base_id = sima_id_ % 10; // 取出 1, 2, 3, 4
        auto elapsed = (this->now() - pre_pos_timer_).seconds();
        geometry_msgs::msg::Twist cmd_vel;

        if (base_id == 1 || base_id == 2 || base_id == 3 || base_id == 4) {
            // 1號、2號執行 [前進1 -> 旋轉1 -> 前進2 -> 旋轉2]
            if (pre_pos_step_ == 0) { 
                // Step 0: 第一段前進 (套用 fwd1 參數)
                if (elapsed < pre_pos_fwd1_sec_) {
                    cmd_vel.linear.x = pre_pos_fwd1_speed_ * calculateVelocityRatio(elapsed, pre_pos_fwd1_sec_);
                    cmd_vel_nav_pub_->publish(cmd_vel);
                } else {
                    stopRobot();
                    pre_pos_step_ = 1; pre_pos_timer_ = this->now();
                }
            } else if (pre_pos_step_ == 1) { 
                // Step 1: 第一段旋轉 (套用 spin1 參數)
                if (elapsed < pre_pos_spin1_sec_) {
                    cmd_vel.angular.z = pre_pos_spin1_speed_ * calculateVelocityRatio(elapsed, pre_pos_spin1_sec_);
                    cmd_vel_nav_pub_->publish(cmd_vel);
                } else {
                    stopRobot();
                    pre_pos_step_ = 2; pre_pos_timer_ = this->now();
                }
            } else if (pre_pos_step_ == 2) { 
                // Step 2: 第二段前進 (套用 fwd2 參數)
                if (elapsed < pre_pos_fwd2_sec_) {
                    cmd_vel.linear.x = pre_pos_fwd2_speed_ * calculateVelocityRatio(elapsed, pre_pos_fwd2_sec_);
                    cmd_vel_nav_pub_->publish(cmd_vel);
                } else {
                    stopRobot();
                    pre_pos_step_ = 3; pre_pos_timer_ = this->now();
                }
            } else if (pre_pos_step_ == 3) { 
                // Step 3: 第二段旋轉 (套用 spin2 參數)
                if (elapsed < pre_pos_spin2_sec_) {
                    cmd_vel.angular.z = pre_pos_spin2_speed_ * calculateVelocityRatio(elapsed, pre_pos_spin2_sec_);
                    cmd_vel_nav_pub_->publish(cmd_vel);
                } else {
                    stopRobot();
                    RCLCPP_INFO(this->get_logger(), "Pre-Positioning Sequence Finished. Waiting for Start.");
                    current_state_ = State::IDLE; // 恢復 IDLE 準備接 Goal
                }
            }
        } 
        // else {
        //     // 3號、4號執行 [僅一次前進]
        //     // 直接共用第一段前進的參數 (fwd1)
        //     if (pre_pos_step_ == 0) {
        //         if (elapsed < pre_pos_fwd1_sec_) {
        //             cmd_vel.linear.x = pre_pos_fwd1_speed_ * calculateVelocityRatio(elapsed, pre_pos_fwd1_sec_);
        //             cmd_vel_pub_->publish(cmd_vel);
        //         } else {
        //             stopRobot();
        //             RCLCPP_INFO(this->get_logger(), "Pre-Positioning Finished. Waiting for Start.");
        //             current_state_ = State::IDLE;
        //         }
        //     }
        // }
    }
    else if (current_state_ == State::DELAYING) {
        double start_delay = this->get_parameter("start_delay_seconds").as_double();
        if ((this->now() - delay_start_time_).seconds() >= start_delay) {
            RCLCPP_INFO(this->get_logger(), "Delay finished. Starting sprinting phase...");
            current_state_ = State::SPRINTING;
            sprint_start_time_ = this->now();
        }
    } 
    else if (current_state_ == State::SPRINTING) {
        auto elapsed = this->now() - sprint_start_time_;
        if (elapsed.seconds() < sprint_duration_sec_) {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = sprint_speed_ * calculateVelocityRatio(elapsed.seconds(), sprint_duration_sec_);
            cmd_vel.angular.z = 0.0;
            cmd_vel_nav_pub_->publish(cmd_vel);
        } else {
            stopRobot();
            if (sima_id_ == 4 || sima_id_ == 14) {
                RCLCPP_INFO(this->get_logger(), "Phase 1 (Forward) completed. Starting Phase 2 (Spin Right)...");
                current_state_ = State::SEQ_SPIN;
                seq_start_time_ = this->now();
            } else {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Sprint phase completed. Holding zero velocity for %.2f seconds before Nav2...",
                    nav_handoff_stop_duration_sec_);
                current_state_ = State::NAV_HANDOFF_STOP;
                nav_handoff_start_time_ = this->now();
            }
            // RCLCPP_INFO(this->get_logger(), "Sprint phase completed. Starting navigation phase...");
            // current_state_ = State::NAVIGATING;
            // executeMission();
        }
    }
    else if (current_state_ == State::NAV_HANDOFF_STOP) {
        stopRobot();
        if ((this->now() - nav_handoff_start_time_).seconds() >= nav_handoff_stop_duration_sec_) {
            RCLCPP_INFO(this->get_logger(), "Navigation handoff stop completed. Starting navigation phase...");
            current_state_ = State::NAVIGATING;
            executeMission();
        }
    }
    // 【新增】4 號機第二階段：自轉 (加上梯形曲線防打滑)
    else if (current_state_ == State::SEQ_SPIN) {
        auto elapsed = this->now() - seq_start_time_;
        if (elapsed.seconds() < seq_spin_duration_sec_) {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            
            // 角速度套用平滑曲線
            cmd_vel.angular.z = seq_spin_speed_ * calculateVelocityRatio(elapsed.seconds(), seq_spin_duration_sec_); 
            
            cmd_vel_nav_pub_->publish(cmd_vel);
        } else {
            stopRobot();
            RCLCPP_INFO(this->get_logger(), "Phase 2 (Spin) completed. Starting Phase 3 (Forward 2)...");
            current_state_ = State::SEQ_FORWARD_2;
            seq_start_time_ = this->now();
        }
    }
    // 【新增】4 號機第三階段：前進 (加上梯形曲線防打滑)
    else if (current_state_ == State::SEQ_FORWARD_2) {
        auto elapsed = this->now() - seq_start_time_;
        if (elapsed.seconds() < seq_forward2_duration_sec_) {
            geometry_msgs::msg::Twist cmd_vel;
            
            // 線速度套用平滑曲線
            cmd_vel.linear.x = seq_forward2_speed_ * calculateVelocityRatio(elapsed.seconds(), seq_forward2_duration_sec_);
            cmd_vel.angular.z = 0.0;
            
            cmd_vel_nav_pub_->publish(cmd_vel);
        } else {
            stopRobot();
            RCLCPP_INFO(this->get_logger(), "Sima 4 Fixed Sequence Completed Successfully!");
            current_state_ = State::END;
        }
    }
    else if (current_state_ == State::STALL_DETECTED) {
        stopRobot();
        if (current_goal_handle_) {
            RCLCPP_INFO(this->get_logger(), "Executing Cancel Goal from safe thread...");
            nav_client_->async_cancel_goal(current_goal_handle_);
        } else {
            RCLCPP_ERROR(this->get_logger(), "No goal handle to cancel! Jumping to retry...");
            current_state_ = State::RETRY_DELAY;
            retry_start_time_ = this->now();
            return;
        }
        // 切換狀態，防止重複發送 cancel 指令
        current_state_ = State::WAITING_FOR_CANCEL;
    }
    // 等待 resultCallback 傳回 CANCELED
    else if (current_state_ == State::WAITING_FOR_CANCEL) {
        stopRobot(); 
    }
    // 正式的冷卻與重試階段
    else if (current_state_ == State::RETRY_DELAY) {
        stopRobot();
        // 建議冷卻時間設為 2.0 秒，確保 Nav2 內部完全釋放舊路徑的記憶體
        if ((this->now() - retry_start_time_).seconds() >= 2.0) {
            RCLCPP_INFO(this->get_logger(), "Cooldown finished. Retrying navigation...");
            current_state_ = State::NAVIGATING;
            executeMission();
        }
    }
    else if (current_state_ == State::END) {
        stopRobot();
    }
}

void SimaNavigator::stopRobot()
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_nav_pub_->publish(cmd_vel);
    cmd_vel_nav_pub_->publish(cmd_vel);
    cmd_vel_pub_->publish(cmd_vel);
    cmd_vel_pub_->publish(cmd_vel);
}

double SimaNavigator::calculateVelocityRatio(double elapsed_sec, double total_sec)
{
    // 梯形速度曲線 (Trapezoidal Velocity Profile)
    // 假設前後 30% 的時間用於加減速，最少給予 30% 的基礎動力克服靜摩擦力
    double ramp_time = total_sec * 0.3;
    double min_ratio = 0.3;
    
    if (elapsed_sec < ramp_time) {
        // 加速段 (Accel)
        return min_ratio + (1.0 - min_ratio) * (elapsed_sec / ramp_time);
    } else if (elapsed_sec > total_sec - ramp_time) {
        // 減速段 (Decel)
        double remaining_sec = total_sec - elapsed_sec;
        return min_ratio + (1.0 - min_ratio) * (remaining_sec / ramp_time);
    } else {
        // 勻速段 (Cruise)
        return 1.0;
    }
}

std::vector<std::pair<double, double>> SimaNavigator::parseWaypoints(const std::vector<double>& flat_points){
    std::vector<std::pair<double, double>> points;
    if (flat_points.size() % 2 != 0) {
        RCLCPP_ERROR(this->get_logger(), "Waypoints parameter size must be even! (x, y pairs)");
        return points;
    }
    for (size_t i = 0; i < flat_points.size(); i += 2) {
        points.push_back({flat_points[i], flat_points[i+1]});
    }
    return points;
}

// (保留完整的 executeMission 及其他 helper functions)
void SimaNavigator::executeMission()
{
    is_navigating_ = true;

    // 1. Get Robot Current Pose
    geometry_msgs::msg::TransformStamped t;
    double robot_x, robot_y;

    try {
        t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        robot_x = t.transform.translation.x;
        robot_y = t.transform.translation.y;
        RCLCPP_INFO(this->get_logger(), "Robot Current Position: (%.2f, %.2f)", robot_x, robot_y);
    } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not get robot pose: %s", ex.what());
        is_navigating_ = false;
        current_state_ = State::IDLE;
        return;
    }

    // 2. Read in waypoints from parameter server
    std::string base_param = target_pantry_ + "_waypoints";
    std::string waypoints_param_name = base_param; // 預設為常規

    if (current_mission_ == MissionType::AGGRESSIVE && 
              (target_status_ == PantryStatus::ENEMY || target_status_ == PantryStatus::CANATTACK || target_status_ == PantryStatus::EVEN)) {
        waypoints_param_name = target_pantry_ + "_waypoints_aggressive";
    } else if (current_mission_ == MissionType::NORMAL && 
              (target_status_ == PantryStatus::ENEMY || target_status_ == PantryStatus::CANATTACK || target_status_ == PantryStatus::EVEN)) {
        // 如果是 Normal 且是敵方的盤子，選用稍微深一點的路徑
        waypoints_param_name = target_pantry_ + "_waypoints_normal";
    }

    // 確保參數存在
    if (!this->has_parameter(waypoints_param_name)) {
        this->declare_parameter(waypoints_param_name, std::vector<double>{});
    }
    
    std::vector<double> flat_waypoints = this->get_parameter(waypoints_param_name).as_double_array();

    // 【強健性防護網 Fallback】：如果你在 YAML 忘了寫 _deep，就降級用一般的
    if (flat_waypoints.empty() && waypoints_param_name != base_param) {
        RCLCPP_WARN(this->get_logger(), "Waypoints [%s] is empty/missing! Falling back to [%s].", 
                    waypoints_param_name.c_str(), base_param.c_str());
        
        if (!this->has_parameter(base_param)) {
            this->declare_parameter(base_param, std::vector<double>{});
        }
        flat_waypoints = this->get_parameter(base_param).as_double_array();
    }

    std::vector<std::pair<double, double>> raw_points = parseWaypoints(flat_waypoints);
    if (raw_points.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Selected waypoint set is totally empty! Aborting.");
        is_navigating_ = false;
        current_state_ = State::IDLE;
        return;
    }

    const double final_goal_dist =
        std::hypot(raw_points.back().first - robot_x, raw_points.back().second - robot_y);
    RCLCPP_INFO(
        this->get_logger(),
        "Mission waypoints selected: param=%s, count=%zu, final=(%.2f, %.2f), straight_line_from_sprint_end=%.2f m",
        waypoints_param_name.c_str(), raw_points.size(),
        raw_points.back().first, raw_points.back().second, final_goal_dist);

    // 3. 【動態配置 Nav2 參數】
    auto ctrl_msg = std_msgs::msg::String();
    auto plan_msg = std_msgs::msg::String();
    std::vector<rclcpp::Parameter> smoother_params;
    std::vector<rclcpp::Parameter> costmap_params; // 【新增】Costmap 參數陣列

    if (current_mission_ == MissionType::AGGRESSIVE) {
        ctrl_msg.data = "Diff_Aggressive";
        plan_msg.data = "GridBased_Aggressive";
        // 激進派速度配置 (請依實車狀況微調)
        smoother_params = {
            rclcpp::Parameter("max_velocity", std::vector<double>{0.6, 0.0, 30.0}),
            rclcpp::Parameter("max_accel", std::vector<double>{1.0, 0.5, 150.0})
        };
        costmap_params = {
            rclcpp::Parameter("inflation_layer.inflation_radius", 0.05),
            rclcpp::Parameter("inflation_layer.cost_scaling_factor", 5.0) // 降低坡度陡峭感
        };
    } else {
        ctrl_msg.data = "Diff";
        plan_msg.data = "GridBased_Pease";
        // 保守派速度配置
        smoother_params = {
            rclcpp::Parameter("max_velocity", std::vector<double>{0.55, 0.44, 30.0}),
            rclcpp::Parameter("max_accel", std::vector<double>{0.25, 0.25, 150.0})
        };
        costmap_params = {
            rclcpp::Parameter("inflation_layer.inflation_radius", 0.12),
            rclcpp::Parameter("inflation_layer.cost_scaling_factor", 20.0)
        };
    }

    // 發布切換指令給 Nav2 Behavior Tree
    controller_pub_->publish(ctrl_msg);
    planner_pub_->publish(plan_msg);
    RCLCPP_INFO(this->get_logger(), "Configured Nav2 - Planner: %s, Controller: %s", plan_msg.data.c_str(), ctrl_msg.data.c_str());

    // 非同步發送 Velocity Smoother 參數更新
    if (vel_smoother_param_client_->wait_for_service(std::chrono::milliseconds(500))) {
        vel_smoother_param_client_->set_parameters(smoother_params,
            [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
                for (auto & result : future.get()) {
                    if (!result.successful) {
                        RCLCPP_WARN(this->get_logger(), "Failed to update Velocity Smoother param: %s", result.reason.c_str());
                    }
                }
            });
    } else {
        RCLCPP_WARN(this->get_logger(), "Velocity Smoother param service not available. Speeds remain unchanged.");
    }

    const bool enable_dynamic_costmap_inflation =
        this->get_parameter("enable_dynamic_costmap_inflation").as_bool();
    if (!enable_dynamic_costmap_inflation) {
        RCLCPP_INFO(
            this->get_logger(),
            "Skipping dynamic Global Costmap inflation update; using YAML/static parameters.");
    }
    // 【新增】：非同步發送 Global Costmap 參數更新
    else if (global_costmap_param_client_->wait_for_service(std::chrono::milliseconds(500))) {
        const auto costmap_param_request_time = this->now();
        RCLCPP_INFO(
            this->get_logger(),
            "Requesting Global Costmap inflation update: radius=%.3f, scaling=%.3f",
            costmap_params[0].as_double(), costmap_params[1].as_double());
        global_costmap_param_client_->set_parameters(costmap_params,
            [this, costmap_param_request_time](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
                const double elapsed = (this->now() - costmap_param_request_time).seconds();
                for (auto & result : future.get()) {
                    if (!result.successful) {
                        RCLCPP_WARN(
                            this->get_logger(),
                            "Failed to update Costmap param after %.3f s: %s",
                            elapsed, result.reason.c_str());
                    } else {
                        RCLCPP_INFO(
                            this->get_logger(),
                            "Global Costmap inflation dynamically adjusted after %.3f s!",
                            elapsed);
                    }
                }
            });
    } else {
        RCLCPP_WARN(this->get_logger(), "Global Costmap param service not available. Costmap remains unchanged.");
    }

    // rclcpp::sleep_for(std::chrono::milliseconds(50));

    // // 4. Check Costmap and Generate Path
    // {
    //     std::lock_guard<std::mutex> lock(map_mutex_);
    //     if (!latest_costmap_) {
    //         RCLCPP_ERROR(this->get_logger(), "No Global Costmap received yet! Aborting.");
    //         is_navigating_ = false;
    //         current_state_ = State::IDLE;
    //         return;
    //     }
    // }

    auto goal_msg = NavThroughPoses::Goal();
    goal_msg.poses.clear();

    RCLCPP_INFO(this->get_logger(), "Sending pure waypoints to Nav2...");

    for (const auto& pt : raw_points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = pt.first;
        pose.pose.position.y = pt.second;
        // 如果這個點不需要特定的朝向，設為 1.0 即可
        pose.pose.orientation.w = 1.0; 
        
        goal_msg.poses.push_back(pose);
    }

    RCLCPP_INFO(this->get_logger(), "Final waypoints sent to Nav2:");
    for (const auto& pose : goal_msg.poses) {
        RCLCPP_INFO(this->get_logger(), " -> (%.2f, %.2f)", pose.pose.position.x, pose.pose.position.y);
    }

    active_planner_request_ = plan_msg.data;
    active_controller_request_ = ctrl_msg.data;
    active_final_goal_ = goal_msg.poses.back();

    // 5. Send to Action Server
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Nav2 Action Server not available!");
        is_navigating_ = false;
        current_state_ = State::IDLE;
        return;
    }

    auto send_goal_options = rclcpp_action::Client<NavThroughPoses>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&SimaNavigator::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&SimaNavigator::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&SimaNavigator::resultCallback, this, std::placeholders::_1);

    prev_distance_ = -1.0;
    consequtive_distance_zero_ = 0;
    no_path_feedback_active_ = false;
    no_path_timeout_cycles_ = 0;
    RCLCPP_INFO(
        this->get_logger(),
        "Sending NavigateThroughPoses goal: poses=%zu, planner_request=%s, controller_request=%s, final=(%.2f, %.2f), first_path_timeout=%.2fs, grace_cycles=%d",
        goal_msg.poses.size(), plan_msg.data.c_str(), ctrl_msg.data.c_str(),
        active_final_goal_.pose.position.x, active_final_goal_.pose.position.y,
        first_path_timeout_sec_, first_path_grace_cycles_);
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
    int index = my * width + mx;
    
    // Bounds check to avoid segfaults
    if (index >= 0 && index < (width * height)) {
        int8_t cost = latest_costmap_->data[index];
        if (cost >= 0 && cost < 200) {
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            return pose;
        }
        RCLCPP_WARN(this->get_logger(), "Point (%.2f, %.2f) is unsafe (Cost: %d). Searching nearby...", wx, wy, cost);
    }

    int search_radius_cells = static_cast<int>(search_r_m / latest_costmap_->info.resolution);

    for (int r = 1; r < search_radius_cells; ++r) {
        for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                if (std::abs(dx) != r && std::abs(dy) != r) continue;

                int check_x = mx + dx;
                int check_y = my + dy;

                if (check_x >= 0 && check_x < width && check_y >= 0 && check_y < height) {
                    int idx = check_y * width + check_x;
                    int8_t c = latest_costmap_->data[idx];
                   
                    if (c >= 0 && c < 200) {
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
        current_state_ = State::IDLE;
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        current_goal_handle_ = goal_handle;
        no_path_feedback_active_ = false;
        no_path_timeout_cycles_ = 0;
    }
}

void SimaNavigator::feedbackCallback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavThroughPoses::Feedback> feedback)
{
    const bool no_path_feedback = feedback->estimated_time_remaining.sec < 0;
    if (current_state_ == State::NAVIGATING && no_path_feedback) {
        if (!no_path_feedback_active_) {
            no_path_feedback_active_ = true;
            first_no_path_feedback_time_ = this->now();
            RCLCPP_WARN(
                this->get_logger(),
                "Nav2 feedback reports no valid path yet. Starting %.2f second first-path watchdog for planner=%s, controller=%s, final=(%.2f, %.2f).",
                first_path_timeout_sec_,
                active_planner_request_.c_str(), active_controller_request_.c_str(),
                active_final_goal_.pose.position.x, active_final_goal_.pose.position.y);
        } else if ((this->now() - first_no_path_feedback_time_).seconds() >= first_path_timeout_sec_) {
            no_path_timeout_cycles_ += 1;
            if (no_path_timeout_cycles_ <= first_path_grace_cycles_) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Still no valid Nav2 path after %.2f seconds (grace cycle %d/%d). Keeping goal active so BT recovery can clear/replan. robot=(%.2f, %.2f), final=(%.2f, %.2f), distance=%.2f m",
                    first_path_timeout_sec_, no_path_timeout_cycles_, first_path_grace_cycles_,
                    feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y,
                    active_final_goal_.pose.position.x, active_final_goal_.pose.position.y,
                    feedback->distance_remaining);
                first_no_path_feedback_time_ = this->now();
                return;
            }
            RCLCPP_ERROR(
                this->get_logger(),
                "No valid Nav2 path after %.2f seconds plus %d grace cycle(s). Stopping robot and canceling goal for retry. planner=%s, controller=%s, robot=(%.2f, %.2f), final=(%.2f, %.2f), distance=%.2f m, poses_remaining=%u, recoveries=%u",
                first_path_timeout_sec_, first_path_grace_cycles_,
                active_planner_request_.c_str(), active_controller_request_.c_str(),
                feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y,
                active_final_goal_.pose.position.x, active_final_goal_.pose.position.y,
                feedback->distance_remaining,
                static_cast<unsigned int>(feedback->number_of_poses_remaining),
                static_cast<unsigned int>(feedback->number_of_recoveries));
            no_path_feedback_active_ = false;
            no_path_timeout_cycles_ = 0;
            current_state_ = State::STALL_DETECTED;
            stopRobot();
        }
    } else if (!no_path_feedback) {
        no_path_feedback_active_ = false;
        no_path_timeout_cycles_ = 0;
    }

    static int log_counter = 0;
    if (log_counter++ % 20 == 0) { 
        RCLCPP_INFO(
            this->get_logger(),
            "Nav feedback: distance=%.2f m, poses_remaining=%u, recoveries=%u, no_path=%s, robot=(%.2f, %.2f)",
            feedback->distance_remaining,
            static_cast<unsigned int>(feedback->number_of_poses_remaining),
            static_cast<unsigned int>(feedback->number_of_recoveries),
            no_path_feedback ? "true" : "false",
            feedback->current_pose.pose.position.x,
            feedback->current_pose.pose.position.y);

        if (feedback->distance_remaining <= 0.001 && feedback->number_of_poses_remaining > 0) {
            consequtive_distance_zero_ += 1;
            RCLCPP_WARN(
                this->get_logger(),
                "Suspicious zero/near-zero distance while %u pose(s) remain. consecutive_zero_feedback=%d",
                static_cast<unsigned int>(feedback->number_of_poses_remaining),
                consequtive_distance_zero_);
        } else {
            consequtive_distance_zero_ = 0;
        }
        prev_distance_ = feedback->distance_remaining;
        // if (prev_distance_ == 0.0 && feedback->distance_remaining == 0.0) {
        //     consequtive_distance_zero_  += 1;
        // }
        // else if (feedback->distance_remaining > 0.0) {
        //     consequtive_distance_zero_ = 0;
        // }
        // prev_distance_ = feedback->distance_remaining;

        // if (consequtive_distance_zero_ > 10) {
        //     RCLCPP_WARN(this->get_logger(), "Stall detected in feedback! Handing over to control loop...");
        //     consequtive_distance_zero_ = 0;
        //     // 【致命修復】：改變狀態交接給 controlLoop，避免死鎖
        //     current_state_ = State::STALL_DETECTED;
        // }
    }
}

void SimaNavigator::resultCallback(const GoalHandleNav::WrappedResult & result)
{
    is_navigating_ = false;
    // current_state_ = State::END;
    current_goal_handle_.reset(); // 清空 Handle

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Mission Completed Successfully!");
            current_state_ = State::END;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Mission Aborted");
            current_state_ = State::RETRY_DELAY;
            retry_start_time_ = this->now();
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Mission Canceled");
            // if (current_state_ != State::RETRY_DELAY) {
            //     current_state_ = State::END;
            // }
            current_state_ = State::RETRY_DELAY;
            retry_start_time_ = this->now();
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            current_state_ = State::END;
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
