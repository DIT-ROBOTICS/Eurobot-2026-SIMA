#ifndef SIMA_PANTRY_LAYER_HPP_
#define SIMA_PANTRY_LAYER_HPP_

#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "std_msgs/msg/string.hpp"

namespace sima_pantry_costmap_plugin
{

// define pantry structure
struct PantryZone {
  std::string name;
  double min_x, min_y, max_x, max_y;
  bool is_open = false; // Can't go through
};

class SimaPantryLayer : public nav2_costmap_2d::Layer
{
public:
  SimaPantryLayer();
  virtual ~SimaPantryLayer() = default;

  virtual void onInitialize() override;
  
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;
    
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  virtual void reset() override;
  virtual bool isClearable() override { return false; }

private:
  void goalCallback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_sub_;
  std::unordered_map<std::string, PantryZone> pantries_;
  std::mutex mutex_;
  
  int sima_id_;
};

}  // namespace sima_pantry_costmap_plugin

#endif  // SIMA_PANTRY_LAYER_HPP_