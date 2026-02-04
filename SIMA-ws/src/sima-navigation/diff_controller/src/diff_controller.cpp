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
        rclcpp::Time current_time = clock_->now();

        // Transform global plan to the robot's frame and prune already passed poses
        auto transformed_plan = transformGlobalPlan(pose);

        auto it = std::find_if(
            transformed_plan.poses.begin(), transformed_plan.poses.end(),
            [&](const geometry_msgs::msg::PoseStamped & plan_pose){
                return hypot(
                    plan_pose.pose.position.x ,
                    plan_pose.pose.position.y) >= lookahead_dist_;
            });
        
        if (it == transformed_plan.poses.end()){
            it = std::prev(transformed_plan.poses.end());
        }

        const auto & goal_pose = it->pose;

        if (debug_lookahead_pub_->is_activated()){
            geometry_msgs::msg::PoseStamped lookahead_msg;
            lookahead_msg.header = transformed_plan.header;
            lookahead_msg.pose = goal_pose;
            debug_lookahead_pub_->publish(lookahead_msg);
        }

        // Check for obstacles at the lookahead point
        std::string costmap_frame = costmap_ros_->getGlobalFrameID();
        
        
        geometry_msgs::msg::PoseStamped goal_pose_local;
        goal_pose_local.header.frame_id = costmap_ros_->getBaseFrameID();
        goal_pose_local.header.stamp = current_time;
        goal_pose_local.pose = goal_pose; 

        geometry_msgs::msg::PoseStamped goal_pose_global;

        bool is_obstacle = false;
        double linear_vel = 0.0;
        double angular_vel = 0.0;

        
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
                    linear_vel = 0.0;
                    angular_vel = 0.0;
                }
            }
        } else {
             RCLCPP_WARN(logger_, "[%s] Failed to transform lookahead point to costmap frame for checking", plugin_name_.c_str());
        }

        // double linear_vel = 0.0, angular_vel = 0.0;

        if (!is_obstacle){

            const double x = goal_pose.position.x;
            const double y = goal_pose.position.y;
            const double denom = (x * x + y * y);

            // lookahead point heading error (in base frame)
            const double heading_error = std::atan2(y, x);

            // 1) If the heading error is too large: rotate in place until it's acceptable before moving forward (to avoid starting with an arc/erratic turn)
            if (std::fabs(heading_error) > heading_rotate_threshold_) {
                linear_vel = 0.0;
                angular_vel = clampAbs(heading_kp_ * heading_error, std::fabs(max_angular_vel_));
            } else {
                // 2) If the heading error is not large: allow forward movement, but scale linear velocity based on the error (the more skewed, the slower, making it easier to go straight)
                double scale = 1.0;
                if (std::fabs(heading_error) > heading_slowdown_threshold_) {
                    scale =
                    1.0 - (std::fabs(heading_error) - heading_slowdown_threshold_) /
                            std::max(heading_rotate_threshold_ - heading_slowdown_threshold_, 1e-6);
                    scale = std::clamp(scale, 0.0, 1.0);
                }

                linear_vel = std::max(min_turning_linear_vel_, scale * desired_linear_vel_);

                // 3) Slow down when approaching the final goal (using your original approach mechanism)
                const auto & final_pose = transformed_plan.poses.back().pose;
                const double dist_to_goal = hypot(final_pose.position.x, final_pose.position.y);
                if (dist_to_goal < approach_dist_) {
                    const double ratio = std::max(0.0, dist_to_goal / std::max(approach_dist_, 1e-6));
                    linear_vel = std::max(min_approach_linear_vel_, ratio * linear_vel);
                }

                // 4) Enforce minimum linear velocity (if you want to keep it)
                linear_vel = std::max(min_linear_vel_, linear_vel);

                // 5) Angular velocity: use heading P control (prioritize straight line), can add some curvature as auxiliary tracking
                double curvature = 0.0;
                if (denom > 1e-6) {
                    curvature = 2.0 * y / denom;
                }

                // Mix: heading dominant, curvature auxiliary (adjustable between 0.0~0.5)
                // const double curvature_weight = 0.2;
                angular_vel = heading_kp_ * heading_error + curvature_weight_ * (curvature * linear_vel);
                angular_vel = clampAbs(angular_vel, std::fabs(max_angular_vel_));
            }
        }

        // Apply acceleration limits
        double target_linear_vel = linear_vel;
        double target_angular_vel = angular_vel;

        if (last_time_.nanoseconds() != 0) {
            double dt = (current_time - last_time_).seconds();
            
            if (dt > 0.0 && dt < 0.5) {
                // // Linear velocity acceleration limiting
                // double dv_linear = target_linear_vel - last_linear_vel_;
                
                // //  If braking, allow larger deceleration
                // double linear_limit = (dv_linear > 0) ? max_acc_linear_ : max_decel_linear_;
                // dv_linear = std::max(-linear_limit * dt, std::min(dv_linear, linear_limit * dt));
                // target_linear_vel = last_linear_vel_ + dv_linear;

                // // Angular velocity acceleration limiting
                // double dv_angular = target_angular_vel - last_angular_vel_;
                // double angular_limit = (dv_angular > 0) ? max_acc_angular_ : max_decel_angular_;
                // dv_angular = std::max(-angular_limit * dt, std::min(dv_angular, angular_limit * dt));
                // target_angular_vel = last_angular_vel_ + dv_angular;



                // --- 1. 線性速度限制 (Linear) ---
                double dv_linear = target_linear_vel - last_linear_vel_;
                
                // 判斷邏輯：如果目標速度的絕對值 > 上次速度絕對值，就是在"催油門"(Accel)
                // 否則就是在"踩煞車"(Decel)
                bool linear_is_accel = std::abs(target_linear_vel) > std::abs(last_linear_vel_);
                
                double current_decel_limit;
                if (is_obstacle){
                    current_decel_limit = max_decel_linear_emergency_;
                } else {
                    current_decel_limit = max_decel_linear_;
                }
                
                double linear_limit = linear_is_accel ? max_acc_linear_ : current_decel_limit;
                
                // 限制變化量
                dv_linear = std::clamp(dv_linear, -linear_limit * dt, linear_limit * dt);
                target_linear_vel = last_linear_vel_ + dv_linear;

                // --- 2. 角速度限制 (Angular) ---
                double dv_angular = target_angular_vel - last_angular_vel_;

                // 同樣的邏輯：絕對值變大=加速，絕對值變小=減速
                bool angular_is_accel = std::abs(target_angular_vel) > std::abs(last_angular_vel_);
                double angular_limit = angular_is_accel ? max_acc_angular_ : max_decel_angular_;

                // [特殊情況優化]：當正負號反轉時 (例如從順時針瞬間變逆時針)
                // 這種情況應該視為"全力煞車"，所以強制使用 Decel 限制
                if (target_angular_vel * last_angular_vel_ < 0) {
                    angular_limit = max_decel_angular_;
                }

                // 限制變化量
                dv_angular = std::clamp(dv_angular, -angular_limit * dt, angular_limit * dt);
                target_angular_vel = last_angular_vel_ + dv_angular;
            }
        }

        last_linear_vel_ = target_linear_vel;
        last_angular_vel_ = target_angular_vel;
        last_time_ = current_time;

        // Create and return TwistStamped message
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = pose.header.frame_id;
        cmd_vel.header.stamp = current_time;
        cmd_vel.twist.linear.x = target_linear_vel;
        cmd_vel.twist.linear.y = 0.0;
        cmd_vel.twist.angular.z = target_angular_vel;

        if (is_obstacle && std::abs(target_linear_vel) < 1e-3) {
            last_linear_vel_ = 0.0;
            throw nav2_core::PlannerException("DiffController: Emergency stop due to obstacle detected ahead.");
        }

        return cmd_vel;
    }


}  // namespace diff_controller

PLUGINLIB_EXPORT_CLASS(diff_controller::DiffController, nav2_core::Controller)
