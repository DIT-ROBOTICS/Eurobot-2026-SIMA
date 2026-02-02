#ifndef SENSOR_LAYER_HPP_
#define SENSOR_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <vector>
#include <mutex>

namespace Sensor_costmap_plugin
{

struct ObstacleNode 
{
    double x;
    double y;
    double z;
    rclcpp::Time last_seen_time;
};

struct IgnoreZone
{
    double min_x;
    double min_y;
    double max_x;
    double max_y;
};


class SensorLayer : public nav2_costmap_2d::Layer
{
public:
  SensorLayer();
  virtual ~SensorLayer();

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
  void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  bool isInsideAnyIgnoreZone(double x, double y);
  bool isPointNearOtherRobots(double x, double y);

  void parseIgnoreZones(const std::vector<double>& params);

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  std::vector<IgnoreZone> ignore_zones_list_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  void removeOutdatedObstacles();

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;

  // Declare Subscriber of sima pose
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sima1_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sima2_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sima3_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sima4_pose_sub_;
  
  // Store all historically detected obstacle center points (World Frame / Map Frame)
  std::vector<ObstacleNode> persistent_obstacles_;
  
  std::mutex data_mutex_;

  int sima_id_;
  double obstacle_radius_;
  double inflation_radius_;
  double cost_scaling_factor_;
  double obstacle_lifespan_ = 5.0;

  // Declare variable to store other robots' positions
  geometry_msgs::msg::Pose sima1_pose_;
  geometry_msgs::msg::Pose sima2_pose_;
  geometry_msgs::msg::Pose sima3_pose_;
  geometry_msgs::msg::Pose sima4_pose_;
};

}  // namespace Sensor_costmap_plugin

#endif