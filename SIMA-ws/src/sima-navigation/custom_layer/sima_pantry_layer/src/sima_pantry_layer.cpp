#include "sima_pantry_layer/sima_pantry_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>

PLUGINLIB_EXPORT_CLASS(sima_pantry_costmap_plugin::SimaPantryLayer, nav2_costmap_2d::Layer)

namespace sima_pantry_costmap_plugin
{

SimaPantryLayer::SimaPantryLayer() {}

void SimaPantryLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  // 1. Define and read in parameters
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  declareParameter("sima_id", rclcpp::ParameterValue(1));
  node->get_parameter(name_ + "." + "sima_id", sima_id_);

  // 2. declare and read in pantry names list (例如: ["pantry_A", "pantry_B"])
  std::vector<std::string> pantry_names;
  declareParameter("pantry_names", rclcpp::ParameterValue(std::vector<std::string>()));
  node->get_parameter(name_ + "." + "pantry_names", pantry_names);

  // 3. Read in pantry range parameters according to each pantry's name [min_x, min_y, max_x, max_y]
  for (const auto& p_name : pantry_names) {
    std::vector<double> zone_array;
    declareParameter(p_name + "_zone", rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0, 0.0}));
    node->get_parameter(name_ + "." + p_name + "_zone", zone_array);

    if (zone_array.size() == 4) {
      PantryZone zone;
      zone.name = p_name;
      zone.min_x = zone_array[0];
      zone.min_y = zone_array[1];
      zone.max_x = zone_array[2];
      zone.max_y = zone_array[3];
      zone.is_open = false; // initial state: Can's go through all pantries (Cost)
      
      pantries_[p_name] = zone;
      RCLCPP_INFO(node->get_logger(), "Loaded Pantry: %s", p_name.c_str());
    } else {
      RCLCPP_WARN(node->get_logger(), "Pantry zone %s has invalid array size!", p_name.c_str());
    }
  }

  // 4. Create subscription
  std::string topic_name = "/sima_" + std::to_string(sima_id_) + "/goal";
  goal_sub_ = node->create_subscription<std_msgs::msg::String>(
    topic_name, 10,
    std::bind(&SimaPantryLayer::goalCallback, this, std::placeholders::_1));
    
  current_ = true;
  RCLCPP_INFO(node->get_logger(), "SimaPantryLayer initialized. Listening to %s", topic_name.c_str());
}

void SimaPantryLayer::goalCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::string target_pantry = msg->data;

  // Check whether the received is correct (Is in the vector)
  if (pantries_.find(target_pantry) != pantries_.end()) {
    pantries_[target_pantry].is_open = true;
    RCLCPP_INFO(node_.lock()->get_logger(), "Pantry [%s] is now OPEN (Cost removed)", target_pantry.c_str());
  } else {
    RCLCPP_WARN(node_.lock()->get_logger(), "Received unknown pantry target: %s", target_pantry.c_str());
  }
}

void SimaPantryLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!enabled_) return;

  std::lock_guard<std::mutex> lock(mutex_);
  
  // Add all pantry into update range
  for (const auto& [name, pantry] : pantries_) {
    *min_x = std::min(*min_x, pantry.min_x);
    *min_y = std::min(*min_y, pantry.min_y);
    *max_x = std::max(*max_x, pantry.max_x);
    *max_y = std::max(*max_y, pantry.max_y);
  }
}

void SimaPantryLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;

  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto& [name, pantry] : pantries_) {
    // If the pantry is open, we do nothing
    if (pantry.is_open) continue;

    // Change meter into grid
    unsigned int mx0, my0, mx1, my1;
    if (!master_grid.worldToMap(pantry.min_x, pantry.min_y, mx0, my0) ||
        !master_grid.worldToMap(pantry.max_x, pantry.max_y, mx1, my1)) {
      continue;
    }

    // Check the bound is within updated bounds (from min_i, min_j to max_i, max_j)
    mx0 = std::max(mx0, static_cast<unsigned int>(min_i));
    my0 = std::max(my0, static_cast<unsigned int>(min_j));
    mx1 = std::min(mx1, static_cast<unsigned int>(max_i));
    my1 = std::min(my1, static_cast<unsigned int>(max_j));

    // set cost LETHAL_OBSTACLE
    for (unsigned int j = my0; j < my1; ++j) {
      for (unsigned int i = mx0; i < mx1; ++i) {
        master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
}

void SimaPantryLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& [name, pantry] : pantries_) {
    pantry.is_open = false; // Reset to close
  }
  current_ = false;
}

}  // namespace sima_costmap_plugin