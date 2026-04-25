#include <string>
#include <algorithm>
#include <memory>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "diff_controller/diff_controller.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

PLUGINLIB_EXPORT_CLASS(diff_controller::DiffController, nav2_core::Controller)


namespace diff_controller
{
    // Helper function: clamp angular z
    static double clampAbs (double val, double abs_limit){
        return max(-abs_limit, min(val, abs_limit));
    }

    // Find iterator of element in path closest to pose
    template <typename Iter, typename Getter>
    Iter min_by (Iter begin, Iter end, Getter getVal){
        if (begin == end) return end; 
        auto best = getVal(*begin);
        Iter best_it = begin;
        for (Iter it = std::next(begin); it != end; ++it){
            auto val = getVal(*it);
            if (val < best){
                best = val;
                best_it = it;
            }
        }
        return best_it;
    }

    void DiffController::configure(
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

        // Get parameters from the config file
        declare_parameter_if_not_declared(node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.4));
        declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.3));
        declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(2.0));
        declare_parameter_if_not_declared(node, plugin_name_ + ".min_linear_vel", rclcpp::ParameterValue(0.0));
        declare_parameter_if_not_declared(node, plugin_name_ + ".min_approach_linear_vel", rclcpp::ParameterValue(0.1));
        declare_parameter_if_not_declared(node, plugin_name_ + ".approach_dist", rclcpp::ParameterValue(0.5));
        declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
        declare_parameter_if_not_declared(node, plugin_name_ + ".heading_rotate_threshold", rclcpp::ParameterValue(0.6));
        declare_parameter_if_not_declared(node, plugin_name_ + ".heading_slowdown_threshold", rclcpp::ParameterValue(0.3));
        declare_parameter_if_not_declared(node, plugin_name_ + ".heading_kp", rclcpp::ParameterValue(2.5));
        declare_parameter_if_not_declared(node, plugin_name_ + ".min_turning_linear_vel", rclcpp::ParameterValue(0.05));
        declare_parameter_if_not_declared(node, plugin_name_ + ".max_acc_linear", rclcpp::ParameterValue(0.02));
        declare_parameter_if_not_declared(node, plugin_name_ + ".max_acc_angular", rclcpp::ParameterValue(0.1));
        declare_parameter_if_not_declared(node, plugin_name_ + ".max_decel_linear", rclcpp::ParameterValue(0.04));
        declare_parameter_if_not_declared(node, plugin_name_ + ".max_decel_linear_emergency", rclcpp::ParameterValue(2.0));
        declare_parameter_if_not_declared(node, plugin_name_ + ".max_decel_angular", rclcpp::ParameterValue(0.2));
        declare_parameter_if_not_declared(node, plugin_name_ + ".curvature_weight", rclcpp::ParameterValue(0.2));

        node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
        node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
        node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
        node->get_parameter(plugin_name_ + ".min_linear_vel", min_linear_vel_);
        node->get_parameter(plugin_name_ + ".min_approach_linear_vel", min_approach_linear_vel_);
        node->get_parameter(plugin_name_ + ".approach_dist", approach_dist_);
        node->get_parameter(plugin_name_ + ".heading_rotate_threshold", heading_rotate_threshold_);
        node->get_parameter(plugin_name_ + ".heading_slowdown_threshold", heading_slowdown_threshold_);
        node->get_parameter(plugin_name_ + ".heading_kp", heading_kp_);
        node->get_parameter(plugin_name_ + ".min_turning_linear_vel", min_turning_linear_vel_);
        node->get_parameter(plugin_name_ + ".max_acc_linear", max_acc_linear_);
        node->get_parameter(plugin_name_ + ".max_acc_angular", max_acc_angular_);
        node->get_parameter(plugin_name_ + ".max_decel_linear", max_decel_linear_);
        node->get_parameter(plugin_name_ + ".max_decel_linear_emergency", max_decel_linear_emergency_);
        node->get_parameter(plugin_name_ + ".max_decel_angular", max_decel_angular_);
        node->get_parameter(plugin_name_ + ".curvature_weight", curvature_weight_);


        double transform_tolerance = 0.1;
        node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
        transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

        // Debug publishers (lifecycle publishers)
        debug_global_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(
            "/diff_controller/debug/global_plan", rclcpp::QoS(1));
        debug_lookahead_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/diff_controller/debug/lookahead_point", rclcpp::QoS(1));
        RCLCPP_INFO(logger_, "[%s] Configured controller. v=%.3f lookahead=%.3f wmax=%.3f", plugin_name_.c_str(), desired_linear_vel_, lookahead_dist_, max_angular_vel_);
    }   

    void DiffController::cleanup(){
        RCLCPP_INFO(logger_, "[%s] Cleaning up controller", plugin_name_.c_str());
        debug_global_plan_pub_.reset();
        debug_lookahead_pub_.reset();
    }

    void DiffController::activate(){
        RCLCPP_INFO(logger_, "[%s] Activating controller", plugin_name_.c_str());
        debug_global_plan_pub_->on_activate();
        debug_lookahead_pub_->on_activate();

        last_linear_vel_ = 0.0;
        last_angular_vel_ = 0.0;
        last_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
    }

    void DiffController::deactivate(){
        RCLCPP_INFO(logger_, "[%s] Deactivating controller", plugin_name_.c_str());
        debug_global_plan_pub_->on_deactivate();
        debug_lookahead_pub_->on_deactivate();
    }

    void DiffController::setPlan(const nav_msgs::msg::Path & path)
    {   
        global_plan_ = path;
        if (debug_global_plan_pub_->is_activated()){
            debug_global_plan_pub_->publish(path);
        }
    }

    nav_msgs::msg::Path DiffController::transformGlobalPlan(
            const geometry_msgs::msg::PoseStamped & pose
    ){
        if (global_plan_.poses.empty()){
            throw nav2_core::PlannerException("DiffController: received plan with zero length");
        }

        // Transform global plan to the robot's frame
        geometry_msgs::msg::PoseStamped robot_pose;
        if (!transformPose(tf_, global_plan_.header.frame_id, pose, robot_pose, transform_tolerance_)){
            throw nav2_core::PlannerException("DiffController: Could not transform robot pose into global plan frame");
        }

        // Only keep points within local costmap radius
        nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
        const double dist_threshold = max(
            costmap->getSizeInMetersX(),
            costmap->getSizeInMetersY()) / 2.0;
        
        auto transformation_begin = min_by(
            global_plan_.poses.begin(), global_plan_.poses.end(),
            [&robot_pose](const geometry_msgs::msg::PoseStamped & plan_pose){
                return euclidean_distance(robot_pose, plan_pose);
            });
        
        auto transformation_end = std::find_if(
            transformation_begin, global_plan_.poses.end(),
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

        // Prune passed points
        global_plan_.poses.erase(global_plan_.poses.begin(), transformation_begin);

        if (debug_global_plan_pub_->is_activated()){
            debug_global_plan_pub_->publish(transformed);
        }

        if (transformed.poses.empty()) {
            throw nav2_core::PlannerException("DiffController: transformed plan has 0 poses");
        }

        return transformed;
    }

    bool DiffController::transformPose(
        const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::string & frame,
        const geometry_msgs::msg::PoseStamped & in_pose,
        geometry_msgs::msg::PoseStamped & out_pose,
        const rclcpp::Duration & transform_tolerance) const
        {
        if (in_pose.header.frame_id == frame) {
            out_pose = in_pose;
            return true;
        }

        try {
            tf->transform(in_pose, out_pose, frame);
            return true;
        } catch (tf2::ExtrapolationException &) {
            // fallback to latest available transform if not too old
            auto transform = tf->lookupTransform(frame, in_pose.header.frame_id, tf2::TimePointZero);
            if ((rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) > transform_tolerance) {
            RCLCPP_ERROR(logger_, "DiffController: TF too old converting %s -> %s",
                        in_pose.header.frame_id.c_str(), frame.c_str());
            return false;
            }
            tf2::doTransform(in_pose, out_pose, transform);
            return true;
        } catch (tf2::TransformException & ex) {
            RCLCPP_ERROR(logger_, "DiffController: transformPose exception: %s", ex.what());
            return false;
        }
    }

    void DiffController::setSpeedLimit(const double & /*speed_limit*/, const bool & /*percentage*/)
    {
        return;
    }

    geometry_msgs::msg::TwistStamped DiffController::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker)
    {
        // rclcpp::Time current_time = clock_->now();

        // // Transform global plan to the robot's frame and prune already passed poses
        // auto transformed_plan = transformGlobalPlan(pose);

        // auto it = std::find_if(
        //     transformed_plan.poses.begin(), transformed_plan.poses.end(),
        //     [&](const geometry_msgs::msg::PoseStamped & plan_pose){
        //         return hypot(
        //             plan_pose.pose.position.x ,
        //             plan_pose.pose.position.y) >= lookahead_dist_;
        //     });
        
        // if (it == transformed_plan.poses.end()){
        //     it = std::prev(transformed_plan.poses.end());
        // }

        // const auto & goal_pose = it->pose;

        // if (debug_lookahead_pub_->is_activated()){
        //     geometry_msgs::msg::PoseStamped lookahead_msg;
        //     lookahead_msg.header = transformed_plan.header;
        //     lookahead_msg.pose = goal_pose;
        //     debug_lookahead_pub_->publish(lookahead_msg);
        // }

        // // Check for obstacles at the lookahead point
        // std::string costmap_frame = costmap_ros_->getGlobalFrameID();
        
        
        // geometry_msgs::msg::PoseStamped goal_pose_local;
        // goal_pose_local.header.frame_id = costmap_ros_->getBaseFrameID();
        // goal_pose_local.header.stamp = current_time;
        // goal_pose_local.pose = goal_pose; 

        // geometry_msgs::msg::PoseStamped goal_pose_global;

        // bool is_obstacle = false;
        // double linear_vel = 0.0;
        // double angular_vel = 0.0;

        
        // if (transformPose(tf_, costmap_frame, goal_pose_local, goal_pose_global, transform_tolerance_)) {
            
        //     nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
        //     unsigned int mx, my;

        //     if (costmap->worldToMap(goal_pose_global.pose.position.x, goal_pose_global.pose.position.y, mx, my)) {
        //         unsigned char cost = costmap->getCost(mx, my);

        //         // debug log
        //         // RCLCPP_INFO(logger_, "Check Cost: %d at (%.2f, %.2f)", cost, goal_pose_global.pose.position.x, goal_pose_global.pose.position.y);
        //         if (cost == nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        //             RCLCPP_WARN(logger_, "[%s] Stop! Obstacle detected ahead. Cost: %d", plugin_name_.c_str(), cost);
                    
        //             // last_linear_vel_ = 0.0;
        //             // last_angular_vel_ = 0.0;
        //             // last_time_ = current_time;

        //             // geometry_msgs::msg::TwistStamped stop_cmd;
        //             // stop_cmd.header.stamp = clock_->now();
        //             // stop_cmd.header.frame_id = pose.header.frame_id;
        //             // return stop_cmd;

        //             is_obstacle = true;
        //             linear_vel = 0.0;
        //             angular_vel = 0.0;
        //         }
        //     }
        // } else {
        //      RCLCPP_WARN(logger_, "[%s] Failed to transform lookahead point to costmap frame for checking", plugin_name_.c_str());
        // }

        // // double linear_vel = 0.0, angular_vel = 0.0;

        // if (!is_obstacle){

        //     const double x = goal_pose.position.x;
        //     const double y = goal_pose.position.y;
        //     const double denom = (x * x + y * y);

        //     // lookahead point heading error (in base frame)
        //     const double heading_error = std::atan2(y, x);

        //     // 1) If the heading error is too large: rotate in place until it's acceptable before moving forward (to avoid starting with an arc/erratic turn)
        //     if (std::fabs(heading_error) > heading_rotate_threshold_) {
        //         linear_vel = 0.0;
        //         angular_vel = clampAbs(heading_kp_ * heading_error, std::fabs(max_angular_vel_));
        //     } else {
        //         // 2) If the heading error is not large: allow forward movement, but scale linear velocity based on the error (the more skewed, the slower, making it easier to go straight)
        //         double scale = 1.0;
        //         if (std::fabs(heading_error) > heading_slowdown_threshold_) {
        //             scale =
        //             1.0 - (std::fabs(heading_error) - heading_slowdown_threshold_) /
        //                     std::max(heading_rotate_threshold_ - heading_slowdown_threshold_, 1e-6);
        //             scale = std::clamp(scale, 0.0, 1.0);
        //         }

        //         linear_vel = std::max(min_turning_linear_vel_, scale * desired_linear_vel_);

        //         // 3) Slow down when approaching the final goal (using your original approach mechanism)
        //         const auto & final_pose = transformed_plan.poses.back().pose;
        //         const double dist_to_goal = hypot(final_pose.position.x, final_pose.position.y);
        //         if (dist_to_goal < approach_dist_) {
        //             const double ratio = std::max(0.0, dist_to_goal / std::max(approach_dist_, 1e-6));
        //             linear_vel = std::max(min_approach_linear_vel_, ratio * linear_vel);
        //         }

        //         // 4) Enforce minimum linear velocity (if you want to keep it)
        //         linear_vel = std::max(min_linear_vel_, linear_vel);

        //         // 5) Angular velocity: use heading P control (prioritize straight line), can add some curvature as auxiliary tracking
        //         double curvature = 0.0;
        //         if (denom > 1e-6) {
        //             curvature = 2.0 * y / denom;
        //         }

        //         // Mix: heading dominant, curvature auxiliary (adjustable between 0.0~0.5)
        //         // const double curvature_weight = 0.2;
        //         angular_vel = heading_kp_ * heading_error + curvature_weight_ * (curvature * linear_vel);
        //         angular_vel = clampAbs(angular_vel, std::fabs(max_angular_vel_));
        //     }
        // }

        // // Apply acceleration limits
        // double target_linear_vel = linear_vel;
        // double target_angular_vel = angular_vel;

        // if (last_time_.nanoseconds() != 0) {
        //     double dt = (current_time - last_time_).seconds();
            
        //     if (dt > 0.0 && dt < 0.5) {
        //         // // Linear velocity acceleration limiting
        //         // double dv_linear = target_linear_vel - last_linear_vel_;
                
        //         // //  If braking, allow larger deceleration
        //         // double linear_limit = (dv_linear > 0) ? max_acc_linear_ : max_decel_linear_;
        //         // dv_linear = std::max(-linear_limit * dt, std::min(dv_linear, linear_limit * dt));
        //         // target_linear_vel = last_linear_vel_ + dv_linear;

        //         // // Angular velocity acceleration limiting
        //         // double dv_angular = target_angular_vel - last_angular_vel_;
        //         // double angular_limit = (dv_angular > 0) ? max_acc_angular_ : max_decel_angular_;
        //         // dv_angular = std::max(-angular_limit * dt, std::min(dv_angular, angular_limit * dt));
        //         // target_angular_vel = last_angular_vel_ + dv_angular;



        //         // --- 1. 線性速度限制 (Linear) ---
        //         double dv_linear = target_linear_vel - last_linear_vel_;
                
        //         // 判斷邏輯：如果目標速度的絕對值 > 上次速度絕對值，就是在"催油門"(Accel)
        //         // 否則就是在"踩煞車"(Decel)
        //         bool linear_is_accel = std::abs(target_linear_vel) > std::abs(last_linear_vel_);
                
        //         double current_decel_limit;
        //         if (is_obstacle){
        //             current_decel_limit = max_decel_linear_emergency_;
        //         } else {
        //             current_decel_limit = max_decel_linear_;
        //         }
                
        //         double linear_limit = linear_is_accel ? max_acc_linear_ : current_decel_limit;
                
        //         // 限制變化量
        //         dv_linear = std::clamp(dv_linear, -linear_limit * dt, linear_limit * dt);
        //         target_linear_vel = last_linear_vel_ + dv_linear;

        //         // --- 2. 角速度限制 (Angular) ---
        //         double dv_angular = target_angular_vel - last_angular_vel_;

        //         // 同樣的邏輯：絕對值變大=加速，絕對值變小=減速
        //         bool angular_is_accel = std::abs(target_angular_vel) > std::abs(last_angular_vel_);
        //         double angular_limit = angular_is_accel ? max_acc_angular_ : max_decel_angular_;

        //         // [特殊情況優化]：當正負號反轉時 (例如從順時針瞬間變逆時針)
        //         // 這種情況應該視為"全力煞車"，所以強制使用 Decel 限制
        //         if (target_angular_vel * last_angular_vel_ < 0) {
        //             angular_limit = max_decel_angular_;
        //         }

        //         // 限制變化量
        //         dv_angular = std::clamp(dv_angular, -angular_limit * dt, angular_limit * dt);
        //         target_angular_vel = last_angular_vel_ + dv_angular;
        //     }
        // }

        // last_linear_vel_ = target_linear_vel;
        // last_angular_vel_ = target_angular_vel;
        // last_time_ = current_time;

        // // Create and return TwistStamped message
        // geometry_msgs::msg::TwistStamped cmd_vel;
        // cmd_vel.header.frame_id = pose.header.frame_id;
        // cmd_vel.header.stamp = current_time;
        // cmd_vel.twist.linear.x = target_linear_vel;
        // cmd_vel.twist.linear.y = 0.0;
        // cmd_vel.twist.angular.z = target_angular_vel;

        // if (is_obstacle && std::abs(target_linear_vel) < 1e-3) {
        //     last_linear_vel_ = 0.0;
        //     throw nav2_core::PlannerException("DiffController: Emergency stop due to obstacle detected ahead.");
        // }

        // return cmd_vel;




        rclcpp::Time current_time = clock_->now();

        // Transform path to Robot Frame
        auto transformed_plan = transformGlobalPlan(pose);

        // Adjust Lookahead Distance by current speed (Adaptive Lookahead)
        double current_speed = std::abs(velocity.linear.x);
        // double adaptive_lookahead = std::clamp(current_speed * 0.8, 0.1, lookahead_dist_);
        double adaptive_lookahead = std::clamp(current_speed * lookahead_dist_ / desired_linear_vel_, 0.25, lookahead_dist_);

        // Find Lookahead Point
        auto it = std::find_if(
            transformed_plan.poses.begin(), transformed_plan.poses.end(),
            [&](const geometry_msgs::msg::PoseStamped & plan_pose){
                return hypot(
                    plan_pose.pose.position.x,
                    plan_pose.pose.position.y) >= adaptive_lookahead;
            });
        
        if (it == transformed_plan.poses.end()){
            it = std::prev(transformed_plan.poses.end());
        }

        size_t lookahead_index = std::distance(transformed_plan.poses.begin(), it);
        const auto & goal_pose = it->pose;

        if (debug_lookahead_pub_->is_activated()){
            geometry_msgs::msg::PoseStamped lookahead_msg;
            lookahead_msg.header = transformed_plan.header;
            lookahead_msg.pose = goal_pose;
            debug_lookahead_pub_->publish(lookahead_msg);
        }


        // Calculate Heading Error (under robot frame)
        const double x = goal_pose.position.x;
        const double y = goal_pose.position.y;
        const double heading_error = std::atan2(y, x);

        // Initialize target vel
        double target_linear_vel = 0.0;
        double target_angular_vel = 0.0;
        bool is_rotating_in_place = false;

        // Rotate in Place
        if (std::abs(heading_error) > heading_rotate_threshold_) {
            is_rotating_in_place = true;
            target_linear_vel = 0.0;
            
            // Use p control
            target_angular_vel = 0.8 * heading_kp_ * heading_error;
            
            // Limit max angular velocity
            target_angular_vel = clampAbs(target_angular_vel, max_angular_vel_);
            // target_angular_vel = std::min(target_angular_vel, max_angular_vel_);
            
            // 避免角速度太小導致馬達轉不動 (Deadband)
            double min_rot_vel = 0.1; 
            if (std::abs(target_angular_vel) < min_rot_vel) {
                target_angular_vel = (target_angular_vel > 0) ? min_rot_vel : -min_rot_vel;
            }

        } else {
            // // --- 6. 正常移動邏輯 (Moving Logic) ---
            
            // // A. 計算前方最大曲率，用於減速
            // double max_curvature_ahead = findMaxCurvature(transformed_plan, lookahead_index);
            
            // // B. 基於曲率的速度限制 (Curvature Slowdown)
            // // 公式： v = v_des / (1 + w * k)
            // double curvature_slowdown_factor = 0.5; // 可調整係數，越大過彎越慢
            // double regulated_linear_vel = desired_linear_vel_ / (1.0 + curvature_slowdown_factor * max_curvature_ahead);

            // // C. 基於 Heading Error 的速度限制 (Heading Slowdown)
            // // 當稍微有點偏時，也稍微減速，讓轉向更準
            // if (std::abs(heading_error) > heading_slowdown_threshold_) {
            //      double heading_scale = 1.0 - (std::abs(heading_error) - heading_slowdown_threshold_) / 
            //                                   std::max(heading_rotate_threshold_ - heading_slowdown_threshold_, 1e-6);
            //      regulated_linear_vel *= std::clamp(heading_scale, 0.7, 1.0);
            // }

            // // D. 靠近終點減速 (Approach Slowdown)
            // double dist_to_goal = hypot(transformed_plan.poses.back().pose.position.x, 
            //                             transformed_plan.poses.back().pose.position.y);
            // if (dist_to_goal < approach_dist_) {
            //     regulated_linear_vel = std::min(regulated_linear_vel, 
            //         (dist_to_goal / 2 / approach_dist_) * desired_linear_vel_);
            //     regulated_linear_vel = std::max(regulated_linear_vel, min_approach_linear_vel_);
            // }

            // // 確保不低於最小速度 (除非很靠近終點)
            // if (dist_to_goal > 0.01) { // 5cm 容差
            //     regulated_linear_vel = std::max(regulated_linear_vel, min_turning_linear_vel_);
            // } else {
            //     regulated_linear_vel = 0.0; // 到達終點
            // }
            
            // target_linear_vel = regulated_linear_vel;

            // // // E. 計算角速度 (Kp + Curvature Feedforward)
            // // double dist_sq = x*x + y*y;
            // // double curvature_term = 0.0;
            // // if (dist_sq > 1e-6) {
            // //     // Pure Pursuit Curvature Formula: 2y / L^2
            // //     curvature_term = 2.0 * y / dist_sq;
            // // }

            // // // 混合控制：Heading Kp 為主，Curvature 為輔
            // // target_angular_vel = (heading_kp_ * heading_error) + (curvature_weight_ * curvature_term * target_linear_vel);
            // // target_angular_vel = clampAbs(target_angular_vel, max_angular_vel_);

            // // E. 計算角速度 (改為正統 Pure Pursuit 為主)
            // double dist_sq = x*x + y*y;
            
            // if (dist_sq > 1e-6) {
            //     // Pure Pursuit 角速度公式： omega = (2 * y / L^2) * v
            //     double pure_pursuit_omega = (2.0 * y / dist_sq) * target_linear_vel;
                
            //     // 移除生硬的 Kp 相加，改用標準的純追跡輸出，或在此加入微弱的 Kp 修正(僅在極低速時)
            //     target_angular_vel = pure_pursuit_omega; 

            //     // 若依然希望在直線稍微偏離時加速修正，可採用以下混合（但不建議 Kp 過大）：
            //     // target_angular_vel = pure_pursuit_omega + (heading_kp_ * heading_error * (1.0 - (target_linear_vel/desired_linear_vel_)));
            // } else {
            //     target_angular_vel = 0.0;
            // }

            // target_angular_vel = clampAbs(target_angular_vel, max_angular_vel_);



            // --- 正常移動邏輯 (Moving Logic) ---
            
            // A & B: 利用運動學模型，直接獲取「為了過前方的彎，現在必須維持的安全速度」
            double regulated_linear_vel = calculateDynamicSafeSpeed(transformed_plan);

            // --- 終點減速與停止邏輯 (含定位延遲補償) ---
            
            // 1. 獲取測量到的剩餘距離
            double measured_dist = std::hypot(transformed_plan.poses.back().pose.position.x, 
                                              transformed_plan.poses.back().pose.position.y);
            
            // 2. 預估定位延遲 (0.2秒延遲其實滿大的，若還是太早停可以考慮微調至 0.1~0.15)
            const double localization_delay = 0.2; 
            
            // 3. 計算預測距離 (僅用於觸發停止與緩行，不參與減速曲線計算，避免震盪)
            double current_linear_speed = std::abs(velocity.linear.x);
            double predicted_dist = measured_dist - (current_linear_speed * localization_delay);
            
            // 確保預測距離不會變成負數
            predicted_dist = std::max(0.0, predicted_dist);

            if (measured_dist < approach_dist_) {
                // 【核心修正】：這裡必須用 measured_dist 來算根號！
                // 這樣才能維持恆定減速度的物理特性，確保過程滑順舒適。
                double approach_ratio = std::sqrt(measured_dist / approach_dist_);
                double approach_vel = 0.8 * approach_ratio * desired_linear_vel_;
                
                // 終端緩行區邏輯 (使用 predicted_dist 提早觸發緩行)
                if (predicted_dist < 0.10) {
                    approach_vel = std::min(approach_vel, 0.05); 
                }
                regulated_linear_vel = std::min(regulated_linear_vel, approach_vel);
            }

            // --- 停止判定：當「預測」已經抵達，或是「測量」已經進到極小範圍 ---
            if (predicted_dist > 0.015 && measured_dist > 0.02) { 
                regulated_linear_vel = std::max(regulated_linear_vel, min_turning_linear_vel_);
            } else {
                regulated_linear_vel = 0.0; // 預測抵達，立即停機
            }

            // 提前計算 alpha (預視點夾角)
            double alpha = std::atan2(y, x);

            // 【新增】：出彎防暴衝機制 (Steering-Throttle Linkage)
            // 利用 yaml 中已有的 heading_slowdown_threshold_ 參數
            if (std::abs(alpha) > heading_slowdown_threshold_) {
                // 當夾角超過閾值，開始平滑扣減速度 (最多打 5 折)
                // 這裡的 0.5 是一個平滑區間，代表超過閾值 0.5 rad 時速度會降到最低
                double penalty = (std::abs(alpha) - heading_slowdown_threshold_) / 0.5;
                double scale = std::clamp(1.0 - penalty, 0.5, 1.0); 
                
                regulated_linear_vel *= scale;
            }
            
            target_linear_vel = regulated_linear_vel;

            // // --- E. 尋跡時的角速度修正 (混合 Pure Pursuit 與 P Control) ---
            // double dist_sq = x * x + y * y;
            
            // double pure_pursuit_omega = 0.0;
            // if (dist_sq > 1e-6) {
            //     // 正統純追跡公式 (Feedforward)：根據當下速度與幾何半徑，主動給出完美過彎角速度
            //     pure_pursuit_omega = (2.0 * y / dist_sq) * target_linear_vel;
            // }

            // // 輔助的 Kp 修正 (Feedback)：用來修正車頭微小偏移，確保直線不蛇行
            // // 因為主力已經交給 pure_pursuit_omega，這裡的 Kp 可以大幅降低，避免震盪
            // double forward_tracking_kp = 1.05; 
            
            // // 最終角速度 = 幾何過彎需求 + 航向誤差修正
            // target_angular_vel = pure_pursuit_omega + (forward_tracking_kp * alpha);

            // // 限制輸出範圍
            // target_angular_vel = clampAbs(target_angular_vel, max_angular_vel_);

            // // 高速防震盪衰減
            // double speed_ratio = target_linear_vel / (desired_linear_vel_ + 1e-6);
            // target_angular_vel *= (1.0 - 0.2 * speed_ratio);

            // target_angular_vel = clampAbs(target_angular_vel, max_angular_vel_);

            // --- E. 尋跡時的角速度修正 (抗重規劃震盪版) ---
            double dist_sq = x * x + y * y;
            
            if (dist_sq > 1e-6) {
                // 1. 純追跡基底 (Pure Pursuit) - 負責提供完美的幾何過彎力道
                target_angular_vel = (2.0 * y / dist_sq) * target_linear_vel;
            } else {
                target_angular_vel = 0.0;
            }

            // 2. 移除前一版的 Kp * alpha。
            // (Pure Pursuit 本身就足夠處理微小偏差，疊加 Kp 反而容易過度反應)

            // 3. 【核心修正】：加入主動阻尼 (Active Damping / Derivative Control)
            // velocity.angular.z 是機器人當下真實的轉速 (來自 Odometry)
            // 當機器人擺頭速度過快時，產生反向的抵抗力，像避震器一樣吸收 Overshoot 的能量！
            // 建議範圍 0.2 ~ 0.6，這會讓直線循跡變得異常死沉、穩定。
            double damping_kd = 0.2; 
            target_angular_vel -= damping_kd * velocity.angular.z;

            // 4. 【核心修正】：加入低通濾波器 (Low-Pass Filter / EMA)
            // 專門撫平 Planner 頻繁 replan 造成的「預視點瞬移雜訊」
            const double filter_weight = 0.5; // (0.0~1.0)，0.5 代表新舊指令各佔一半，非常滑順
            if (last_time_.nanoseconds() != 0) {
                target_angular_vel = filter_weight * target_angular_vel + (1.0 - filter_weight) * last_angular_vel_;
            }

            // 高速防震盪衰減 (保留這個好設計，高速時方向盤變重)
            double speed_ratio = target_linear_vel / (desired_linear_vel_ + 1e-6);
            target_angular_vel *= (1.0 - 0.2 * speed_ratio);

            target_angular_vel = clampAbs(target_angular_vel, max_angular_vel_);
        }

        // --- 7. 障礙物檢查與急停 ---
        // (你的障礙物檢查邏輯放在這裡，如果是 Obstacle 則強制設為 0)
        // // Check for obstacles at the lookahead point
        std::string costmap_frame = costmap_ros_->getGlobalFrameID();
        
        geometry_msgs::msg::PoseStamped goal_pose_local;
        goal_pose_local.header.frame_id = costmap_ros_->getBaseFrameID();
        goal_pose_local.header.stamp = current_time;
        goal_pose_local.pose = goal_pose; 

        geometry_msgs::msg::PoseStamped goal_pose_global;

        bool is_obstacle = false;
        if (transformPose(tf_, costmap_frame, goal_pose_local, goal_pose_global, transform_tolerance_)) {
            
            nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
            unsigned int mx, my;

            if (costmap->worldToMap(goal_pose_global.pose.position.x, goal_pose_global.pose.position.y, mx, my)) {
                unsigned char cost = costmap->getCost(mx, my);

                // debug log
                // RCLCPP_INFO(logger_, "Check Cost: %d at (%.2f, %.2f)", cost, goal_pose_global.pose.position.x, goal_pose_global.pose.position.y);
                if (cost == nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                    RCLCPP_WARN(logger_, "[%s] Stop! Obstacle detected ahead. Cost: %d", plugin_name_.c_str(), cost);
                    
                    // last_linear_vel_ = 0.0;
                    // last_angular_vel_ = 0.0;
                    // last_time_ = current_time;

                    // geometry_msgs::msg::TwistStamped stop_cmd;
                    // stop_cmd.header.stamp = clock_->now();
                    // stop_cmd.header.frame_id = pose.header.frame_id;
                    // return stop_cmd;

                    is_obstacle = true;
                    target_linear_vel = 0.0;
                    target_angular_vel = 0.0;
                }
            }
        } else {
             RCLCPP_WARN(logger_, "[%s] Failed to transform lookahead point to costmap frame for checking", plugin_name_.c_str());
        }
        // if (is_obstacle) { target_linear_vel = 0.0; target_angular_vel = 0.0; ... }
        if (is_obstacle){
            target_linear_vel = 0.0;
            target_angular_vel = 0.0;
        }

        // --- 8. 加減速平滑限制 (Slew Rate Limiter) ---
        if (last_time_.nanoseconds() == 0) {
            target_linear_vel = 0.0;
            target_angular_vel = 0.0;
            RCLCPP_INFO(logger_, "[%s] Controller initialized, first command zeroed for safety.", plugin_name_.c_str());
        }
        else {
            double dt = (current_time - last_time_).seconds();
            if (dt > 0.0 && dt < 0.5) {
                
                // Linear Limiter
                double dv_linear = target_linear_vel - last_linear_vel_;
                bool linear_is_accel = std::abs(target_linear_vel) > std::abs(last_linear_vel_);
                
                // 如果是原地自轉狀態，或是遇到障礙物，可能需要更強的減速能力
                double decel_limit = max_decel_linear_; 
                if (is_obstacle) decel_limit = max_decel_linear_emergency_;

                double linear_limit = linear_is_accel ? max_acc_linear_ : decel_limit;
                dv_linear = std::clamp(dv_linear, -linear_limit * dt, linear_limit * dt);
                target_linear_vel = last_linear_vel_ + dv_linear;

                // Angular Limiter
                double dv_angular = target_angular_vel - last_angular_vel_;
                bool angular_is_accel = std::abs(target_angular_vel) > std::abs(last_angular_vel_);
                
                // 如果正負號反轉 (例如從左轉變右轉)，視為減速
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

        return cmd_vel;
    }

    double DiffController::findMaxCurvature(
        const nav_msgs::msg::Path & transformed_plan) // 移除 lookahead_index 參數
    {
        // 往前看 0.5 到 0.6 公尺 (足以覆蓋目前的煞車距離)
        const double inspection_dist = 0.5; 
        double max_kappa = 0.0;
        
        if (transformed_plan.poses.size() < 4) return 0.0; 

        // 基準點：車體當前位置
        auto base_pt = transformed_plan.poses[0].pose.position;

        // 注意這裡：從 index = 2 開始找 (略過0和1避免里程計座標的極小震盪雜訊)
        for (size_t i = 2; i < transformed_plan.poses.size() - 2; ++i) {
            const auto & p1 = transformed_plan.poses[i].pose.position;
            const auto & p2 = transformed_plan.poses[i+1].pose.position;
            const auto & p3 = transformed_plan.poses[i+2].pose.position;
            
            // 計算距離車體的距離
            double dist = std::hypot(p1.x - base_pt.x, p1.y - base_pt.y);
            if (dist > inspection_dist) break; // 超過檢查範圍就停止

            // 計算三邊長
            double a = std::hypot(p2.x - p3.x, p2.y - p3.y);
            double b = std::hypot(p1.x - p3.x, p1.y - p3.y);
            double c = std::hypot(p1.x - p2.x, p1.y - p2.y);

            double cross_product = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
            double area = 0.5 * std::abs(cross_product);

            double denominator = a * b * c;
            if (denominator > 1e-6) {
                double kappa = (4.0 * area) / denominator;
                if (kappa < 15.0 && kappa > max_kappa) { // 稍微放寬上限到 15 (半徑約 6.6cm)
                    max_kappa = kappa;
                }
            }
        }
        return max_kappa;
    }

    // 傳回：當前機器人「現在」應該保持的安全速度
    double DiffController::calculateDynamicSafeSpeed(const nav_msgs::msg::Path & plan)
    {
        double current_safe_speed = desired_linear_vel_;
        
        // --- 物理參數設定 ---
        const double a_lat_max = 0.35; // 最大側向加速度 (越大過彎越快)
        // 為了安全，煞車規劃時只信任 80% 的最大減速度
        const double planning_decel = max_decel_linear_ * 0.4; 
        
        // --- 空間解析度設定 (解決點太密集的問題) ---
        // 強制要求相鄰兩個取樣點至少要間隔 10 公分 (約等於車身長度)
        const double baseline_dist = 0.1; 
        
        // 往前掃描距離：至少要能涵蓋「從極速煞停」的距離 ( 0.55^2 / (2 * 2.2) 約為 0.07m，加上安全餘裕看 0.6m 絕對夠)
        const double max_scan_dist = 0.6; 

        if (plan.poses.size() < 3) return min_turning_linear_vel_;

        auto base_pt = plan.poses[0].pose.position;

        // 沿著路徑往前掃描
        for (size_t i = 0; i < plan.poses.size(); ++i) {
            const auto & p1 = plan.poses[i].pose.position;
            double dist_to_curve = std::hypot(p1.x - base_pt.x, p1.y - base_pt.y);

            if (dist_to_curve > max_scan_dist) break;

            // 尋找距離 p1 至少 baseline_dist 的點 p2
            size_t idx2 = i + 1;
            while (idx2 < plan.poses.size() && 
                   std::hypot(plan.poses[idx2].pose.position.x - p1.x, 
                              plan.poses[idx2].pose.position.y - p1.y) < baseline_dist) {
                idx2++;
            }
            if (idx2 >= plan.poses.size()) break;
            const auto & p2 = plan.poses[idx2].pose.position;

            // 尋找距離 p2 至少 baseline_dist 的點 p3
            size_t idx3 = idx2 + 1;
            while (idx3 < plan.poses.size() && 
                   std::hypot(plan.poses[idx3].pose.position.x - p2.x, 
                              plan.poses[idx3].pose.position.y - p2.y) < baseline_dist) {
                idx3++;
            }
            if (idx3 >= plan.poses.size()) break;
            const auto & p3 = plan.poses[idx3].pose.position;

            // 計算真實幾何曲率 (Menger Curvature)
            double a = std::hypot(p2.x - p3.x, p2.y - p3.y);
            double b = std::hypot(p1.x - p3.x, p1.y - p3.y);
            double c = std::hypot(p1.x - p2.x, p1.y - p2.y);

            double cross_product = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
            double area = 0.5 * std::abs(cross_product);
            double kappa = 0.0;
            double denominator = a * b * c;

            if (denominator > 1e-6) {
                kappa = (4.0 * area) / denominator;
            }

            // 如果該區域有明顯彎道 (曲率大於 0.5 才視為彎道，過濾直線雜訊)
            // 如果該區域有明顯彎道
            if (kappa > 0.5) {
                // 限制 1: 動態極限 (防打滑)
                // 這裡的 dynamic_a_lat_max 可以是你用 mu * g 計算出來的，或手動設定的 0.6
                double v_dynamic = std::sqrt(a_lat_max / kappa);

                // 限制 2: 運動學極限 (防外拋)
                // 注意：我們故意只取 max_angular_vel_ 的 80%~90% 作為可用上限。
                // 為什麼？因為要留一點角速度的「餘裕」給 P Controller 去修正循跡誤差。
                // 如果把可用角速度算得太滿，一但有微小誤差，馬達就沒有額外的出力可以修正了。
                double available_omega = max_angular_vel_ * 0.5; 
                double v_kinematic = available_omega / kappa;

                // 綜合兩者，取最嚴格的安全速度
                double v_curve = std::min(v_dynamic, v_kinematic);
                
                // 確保不低於最小轉彎速度，不超過直線目標速度
                v_curve = std::clamp(v_curve, min_turning_linear_vel_, desired_linear_vel_);

                // 運動學反推：為抵達彎道降至 v_curve，現在的安全速度？
                double v_safe_now = std::sqrt(v_curve * v_curve + 2.0 * planning_decel * dist_to_curve);

                // 更新當前最嚴格的安全速度
                if (v_safe_now < current_safe_speed) {
                    current_safe_speed = v_safe_now;
                }
            }
        }

        return current_safe_speed;
    }


}  // namespace diff_controller

PLUGINLIB_EXPORT_CLASS(diff_controller::DiffController, nav2_core::Controller)
