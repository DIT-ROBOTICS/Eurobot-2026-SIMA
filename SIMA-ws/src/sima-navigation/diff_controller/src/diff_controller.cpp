#include <string>
#include <algorithm>
#include <memory>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"

// #include "nav2_core/planner_exceptions.hpp"
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
        declare_parameter_if_not_declared(node, plugin_name_ + ".min_turing_linear_vel", rclcpp::ParameterValue(0.05));

        node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
        node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
        node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
        node->get_parameter(plugin_name_ + ".min_linear_vel", min_linear_vel_);
        node->get_parameter(plugin_name_ + ".min_approach_linear_vel", min_approach_linear_vel_);
        node->get_parameter(plugin_name_ + ".approach_dist", approach_dist_);
        node->get_parameter(plugin_name_ + ".heading_rotate_threshold", heading_rotate_threshold_);
        node->get_parameter(plugin_name_ + ".heading_slowdown_threshold", heading_slowdown_threshold_);
        node->get_parameter(plugin_name_ + ".heading_kp", heading_kp_);
        node->get_parameter(plugin_name_ + ".min_turing_linear_vel", min_turning_linear_vel_);


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
    }

    void DiffController::deactivate(){
        RCLCPP_INFO(logger_, "[%s] Deactivating controller", plugin_name_.c_str());
        debug_global_plan_pub_->on_deactivate();
        debug_lookahead_pub_->on_deactivate();
    }

    void DiffController::setPlan(const nav_msgs::msg::Path & path)
    {   
        // Transform global path into the robot's frame
        // global_plan_ = path;
        // global_plan_ = transformGlobalPlan(path);

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

        // double linear_vel = 0.0, angular_vel = 0.0;

        // Pure pursuit logic: if goal poitn is in front of robot, compute curvature; 
        // else rotate in place until it is
        // if (goal_pose.position.x > 0){
        //     const double x = goal_pose.position.x;
        //     const double y = goal_pose.position.y;
        //     const double denom = (x * x + y * y);

        //     double curvature = 0.0;
        //     if (denom > 0.0001){
        //         curvature = 2.0 * y / denom;
        //     }

        //     // Base linear velocity on distance to goal point
        //     linear_vel = desired_linear_vel_;

        //     // Simply slow down near final goal
        //     // Use the last point of transformed_plan as "approx final" in robot frame
        //     const auto & final_pose = transformed_plan.poses.back().pose;
        //     const double dist_to_goal = hypot(final_pose.position.x, final_pose.position.y);
        //     if (dist_to_goal < approach_dist_){
        //         // Linearly scale down velocity
        //         const double ratio = max(0.0, dist_to_goal / max(approach_dist_, 0.0001));
        //         linear_vel = max(min_approach_linear_vel_, ratio * desired_linear_vel_);
        //     }

        //     // Enforce min_linear_vel_
        //     linear_vel = max(min_linear_vel_, linear_vel);

        //     // Diff drive: w = curvature * v
        //     angular_vel = curvature * linear_vel;
        // }
        // else {
        //     linear_vel = 0.0;
        //     angular_vel = max_angular_vel_;
        // }

        // // Clamp angular velocity
        // angular_vel = clampAbs(angular_vel, abs(max_angular_vel_));





        // Check for obstacles at the lookahead point
        std::string costmap_frame = costmap_ros_->getGlobalFrameID();
        
        
        geometry_msgs::msg::PoseStamped goal_pose_local;
        goal_pose_local.header.frame_id = costmap_ros_->getBaseFrameID();
        goal_pose_local.header.stamp = clock_->now();
        goal_pose_local.pose = goal_pose; 

        geometry_msgs::msg::PoseStamped goal_pose_global;

        
        if (transformPose(tf_, costmap_frame, goal_pose_local, goal_pose_global, transform_tolerance_)) {
            
            nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
            unsigned int mx, my;

            if (costmap->worldToMap(goal_pose_global.pose.position.x, goal_pose_global.pose.position.y, mx, my)) {
                unsigned char cost = costmap->getCost(mx, my);

                // debug log
                // RCLCPP_INFO(logger_, "Check Cost: %d at (%.2f, %.2f)", cost, goal_pose_global.pose.position.x, goal_pose_global.pose.position.y);
                if (cost == nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                    RCLCPP_WARN(logger_, "[%s] Stop! Obstacle detected ahead. Cost: %d", plugin_name_.c_str(), cost);
                    
                    geometry_msgs::msg::TwistStamped stop_cmd;
                    stop_cmd.header.stamp = clock_->now();
                    stop_cmd.header.frame_id = pose.header.frame_id;
                    return stop_cmd;
                }
            }
        } else {
             RCLCPP_WARN(logger_, "[%s] Failed to transform lookahead point to costmap frame for checking", plugin_name_.c_str());
        }




        double linear_vel = 0.0, angular_vel = 0.0;

        const double x = goal_pose.position.x;
        const double y = goal_pose.position.y;
        const double denom = (x * x + y * y);

        // lookahead 點方向誤差（在 base frame 下）
        const double heading_error = std::atan2(y, x);

        // 1) 若朝向差太大：先原地轉到差不多再走（避免起步先畫弧線/亂轉）
        if (std::fabs(heading_error) > heading_rotate_threshold_) {
            linear_vel = 0.0;
            angular_vel = clampAbs(heading_kp_ * heading_error, std::fabs(max_angular_vel_));
        } else {
            // 2) 朝向差不大：允許前進，但依誤差縮放線速（越歪越慢，越容易直線）
            double scale = 1.0;
            if (std::fabs(heading_error) > heading_slowdown_threshold_) {
                scale =
                1.0 - (std::fabs(heading_error) - heading_slowdown_threshold_) /
                        std::max(heading_rotate_threshold_ - heading_slowdown_threshold_, 1e-6);
                scale = std::clamp(scale, 0.0, 1.0);
            }

            linear_vel = std::max(min_turning_linear_vel_, scale * desired_linear_vel_);

            // 3) 接近終點時的降速（沿用你原本 approach 機制）
            const auto & final_pose = transformed_plan.poses.back().pose;
            const double dist_to_goal = hypot(final_pose.position.x, final_pose.position.y);
            if (dist_to_goal < approach_dist_) {
                const double ratio = std::max(0.0, dist_to_goal / std::max(approach_dist_, 1e-6));
                linear_vel = std::max(min_approach_linear_vel_, ratio * linear_vel);
            }

            // 4) enforce 最小線速（如果你要保留）
            linear_vel = std::max(min_linear_vel_, linear_vel);

            // 5) 角速度：用 heading P 控制（直線優先），可加一點 curvature 作為輔助追蹤
            double curvature = 0.0;
            if (denom > 1e-6) {
                curvature = 2.0 * y / denom;
            }

            // 混合：heading 主導、curvature 輔助（0.0~0.5 之間可調）
            const double curvature_weight = 0.2;
            angular_vel = heading_kp_ * heading_error + curvature_weight * (curvature * linear_vel);
            angular_vel = clampAbs(angular_vel, std::fabs(max_angular_vel_));
        }










        // Create and return TwistStamped message
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = pose.header.frame_id;
        cmd_vel.header.stamp = clock_->now();
        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.linear.y = 0.0;
        cmd_vel.twist.angular.z = angular_vel;
        return cmd_vel;


        // // Find the first pose which is at a distance greater than lookahead_distance_
        // auto goal_pose = std::find_if(global_plan_.poses.begin(), global_plan_.poses.end(),
        //     [&](const auto & global_plan_pose){
        //         return hypot(
        //             global_plan_pose.pose.position.x ,
        //             global_plan_pose.pose.position.y) >= lookahead_dist_;
        //     }) -> pose;
        
        // double linear_vel, angular_vel;
        // // Compute linear and angular velocities to reach goal_pose
        // // If the goal pose is in front of the robot then compute the velocity using the pure pursuit algorithm
        // // else rotate with the max angular velocity until the goal pose is in front of the robot
        // if (goal_pose.position.x > 0){
        //     auto curvature = 2 * goal_pose.position.y / 
        //         (goal_pose.position.x * goal_pose.position.x + goal_pose.position.y * goal_pose.position.y);
        //     linear_vel = desired_linear_vel_;
        //     angular_vel = curvature * linear_vel;
        // }
        // else {
        //     linear_vel = 0.0;
        //     angular_vel = max_angular_vel_;
        // }

        // // Create and publish a TwistStamped message with the desired velocity
        // geometry_msgs::msg::TwistStamped cmd_vel;
        // cmd_vel.header.frame_id = pose.header.frame_id;
        // cmd_vel.header.stamp = clock_->now();
        // cmd_vel.twist.linear.x = linear_vel;
        // cmd_vel.twist.angular.z = max(
        //     -1.0 * abs(max_angular_vel_), 
        //     min(angular_vel, abs(max_angular_vel_))
        // );

        // return cmd_vel;
    }


}  // namespace diff_controller

PLUGINLIB_EXPORT_CLASS(diff_controller::DiffController, nav2_core::Controller)
