#ifndef SIMA_LAYER_HPP_
#define SIMA_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <mutex>

namespace SIMA_costmap_plugin
{
    class SIMALayer : public nav2_costmap_2d::CostmapLayer
    {
        public:
            SIMALayer();
            virtual ~SIMALayer();

            virtual void onInitialize();
            virtual void updateBounds(
                double robot_x, double robot_y, double robot_yaw,
                double * min_x, double * min_y, double * max_x, double * max_y);
            virtual void updateCosts(
                nav2_costmap_2d::Costmap2D & master_grid,
                int min_i, int min_j, int max_i, int max_j);
            // When Costmap is reset (e.g., calling clear_costmaps service), clear memory
            virtual void reset();
            virtual bool isClearable() { return true; }
        
        private:
            // Declare Subscribers
            rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sima1_pose_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sima2_pose_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sima3_pose_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sima4_pose_sub_;

            // Parameters
            double sima_radius_;
            double inflation_radius_;
            double cost_scaling_factor_;
            int sima_id_;
            double tolerance_timeout_;
        
        private:
            // Helper functions
            void ExpandCostWithCircle(double x, double y, double MaxCost, double InflationRadius, double CostScalingFactor, double SimaRadius);
        
        private:
            // Store other simas' poses
            geometry_msgs::msg::Pose sima1_pose_;
            geometry_msgs::msg::Pose sima2_pose_;
            geometry_msgs::msg::Pose sima3_pose_;
            geometry_msgs::msg::Pose sima4_pose_;

            geometry_msgs::msg::Pose last_sima1_pose_;
            geometry_msgs::msg::Pose last_sima2_pose_;
            geometry_msgs::msg::Pose last_sima3_pose_;
            geometry_msgs::msg::Pose last_sima4_pose_;

            bool sima1_pose_received_ = false;
            bool sima2_pose_received_ = false;
            bool sima3_pose_received_ = false;
            bool sima4_pose_received_ = false;

            bool last_sima1_pose_valid_ = false;   // used to mark whether at least one message has been received and initialized
            bool last_sima2_pose_valid_ = false;
            bool last_sima3_pose_valid_ = false;
            bool last_sima4_pose_valid_ = false;

            rclcpp::Time last_seen_sima1_time_;
            rclcpp::Time last_seen_sima2_time_;
            rclcpp::Time last_seen_sima3_time_;
            rclcpp::Time last_seen_sima4_time_;

            std::mutex data_mutex_;
    };

}  // namespace SIMA_costmap_plugin


#endif  // SIMA_LAYER_HPP_