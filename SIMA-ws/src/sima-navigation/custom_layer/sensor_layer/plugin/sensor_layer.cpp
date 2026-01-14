#include "sensor_layer/sensor_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath> 
#include <algorithm>

PLUGINLIB_EXPORT_CLASS(Sensor_costmap_plugin::SensorLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace Sensor_costmap_plugin
{

SensorLayer::SensorLayer() {}
SensorLayer::~SensorLayer() {}

void SensorLayer::onInitialize()
{
  current_ = true;
  enabled_ = true;
  
  auto node = node_.lock(); 
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->declare_parameter(name_ + ".obstacle_radius", 0.15); 
  node->declare_parameter(name_ + ".inflation_radius", 0.2); 
  node->declare_parameter(name_ + ".cost_scaling_factor", 5.0);
  node->declare_parameter(name_ + ".obstacle_lifespan", 5.0);
  node->declare_parameter(name_ + ".ignore_zone_array", std::vector<double>{});
  
  node->get_parameter(name_ + ".obstacle_radius", obstacle_radius_);
  node->get_parameter(name_ + ".inflation_radius", inflation_radius_);
  node->get_parameter(name_ + ".cost_scaling_factor", cost_scaling_factor_);
  node->get_parameter(name_ + ".obstacle_lifespan", obstacle_lifespan_);

  std::vector<double> ignore_zones_raw;
  node->get_parameter(name_ + ".ignore_zone_array", ignore_zones_raw);
  parseIgnoreZones(ignore_zones_raw);

  // 3. Setup dynamic parameter callback (so you can modify yaml without restart, or use rqt_reconfigure to modify)
  dyn_params_handler_ = node->add_on_set_parameters_callback(std::bind(&SensorLayer::dynamicParametersCallback, this, std::placeholders::_1));

  sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
    "/sensors/detected_obstacles", 10,
    std::bind(&SensorLayer::poseArrayCallback, this, std::placeholders::_1));
    
  RCLCPP_INFO(node->get_logger(), "SensorLayer (Persistent Mode) Initialized!");
}

void SensorLayer::parseIgnoreZones(const std::vector<double>& params)
{
    ignore_zones_list_.clear();
    
    // Check if size is a multiple of 4
    if (params.size() % 4 != 0) {
        RCLCPP_ERROR(node_.lock()->get_logger(), 
            "ignore_zones parameter size is %ld, which is not divisible by 4! Ignoring.", params.size());
        return;
    }

    for (size_t i = 0; i < params.size(); i += 4) {
        IgnoreZone zone;
        // Automatically handle min/max order to prevent user input errors
        zone.min_x = std::min(params[i], params[i+2]);
        zone.min_y = std::min(params[i+1], params[i+3]);
        zone.max_x = std::max(params[i+2], params[i]);
        zone.max_y = std::max(params[i+3], params[i+1]);
        ignore_zones_list_.push_back(zone);
    }
}

bool SensorLayer::isInsideAnyIgnoreZone(double x, double y)
{
    for (const auto& zone : ignore_zones_list_) {
        if (x >= zone.min_x && x <= zone.max_x &&
            y >= zone.min_y && y <= zone.max_y) {
            return true;
        }
    }
    return false;
}

rcl_interfaces::msg::SetParametersResult SensorLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock(data_mutex_);

  for (const auto & param : parameters) {
    if (param.get_name() == name_ + ".ignore_zones") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        parseIgnoreZones(param.as_double_array());
        RCLCPP_INFO(node_.lock()->get_logger(), "Updated ignore zones: count = %ld", ignore_zones_list_.size());
      }
    }
    // ... other parameter updates ...
  }

  result.successful = true;
  return result;
}

// This is triggered when someone calls /clear_costmaps
void SensorLayer::reset()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    persistent_obstacles_.clear();
    current_ = false;
}

void SensorLayer::poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Get Costmap's Frame (usually odom)
  // If SensorSim sends in map frame, should we store points as map or odom?
  // Suggestion: To keep obstacles "fixed in the world", we assume SensorSim sends coordinates in "map" frame
  // We can store them directly.

  // Note: This assumes PoseArray's frame_id is consistent with the coordinate system defined for ignore_zones (usually "map")
  // If msg is "base_link" but ignore_zones is "map", there will be a coordinate system mismatch.
  // Suggestion: The sender (Bridge Node) should convert to "map" before sending, or this layer should handle TF conversion.

  auto node = node_.lock();
  if (!node) {
      return;
  }
  rclcpp::Time now = node->now();
  
  for(const auto& new_pose : msg->poses)
  {
      if (isInsideAnyIgnoreZone(new_pose.position.x, new_pose.position.y)) {
          continue; // Ignore this point
      }

      bool is_duplicate = false;
      
      // === Simple spatial filtering ===
      // Check if this new point already exists in our memory (if distance too close, consider it duplicate)
      for(auto& existing_pt : persistent_obstacles_)
      {
          double dist = std::hypot(new_pose.position.x - existing_pt.x, 
                                   new_pose.position.y - existing_pt.y);
          // If new point is less than 7.5cm from old point, don't store it, save performance
          if(dist < 0.075) {
              existing_pt.last_seen_time = now;
              is_duplicate = true;
              break;
          }
      }

      // If it's a new point, store it
      if(!is_duplicate) {
          ObstacleNode pt;
          pt.x = new_pose.position.x;
          pt.y = new_pose.position.y;
          pt.z = new_pose.position.z;
          pt.last_seen_time = now;
          persistent_obstacles_.push_back(pt);
      }
  }
}

void SensorLayer::removeOutdatedObstacles()
{
    // std::lock_guard<std::mutex> lock(data_mutex_);
    auto node = node_.lock();
    if (!node) {
        return;
    }
    rclcpp::Time now = node->now();

    // Remove obstacles that have exceeded their lifespan
    persistent_obstacles_.erase(
        std::remove_if(
            persistent_obstacles_.begin(),
            persistent_obstacles_.end(),
            [&](const ObstacleNode& pt) {
                double age = (now - pt.last_seen_time).seconds();
                return age > obstacle_lifespan_;
            }),
        persistent_obstacles_.end());
}

void SensorLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) return;
  std::lock_guard<std::mutex> lock(data_mutex_);

  removeOutdatedObstacles();
  
  // Iterate through all points in "memory" to update bounds
  double range = inflation_radius_ + 0.05; // Add a bit of buffer

  for(const auto& pt : persistent_obstacles_) {
      *min_x = std::min(*min_x, pt.x - range);
      *min_y = std::min(*min_y, pt.y - range);
      *max_x = std::max(*max_x, pt.x + range);
      *max_y = std::max(*max_y, pt.y + range);
  }
}

void SensorLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!enabled_) return;
  std::lock_guard<std::mutex> lock(data_mutex_);

  std::string costmap_frame = layered_costmap_->getGlobalFrameID();
  auto tf = tf_;

  // Iterate through all obstacle points in "memory"
  for (const auto& pt_map : persistent_obstacles_)
  {
      // 1. TF transformation
      // Because points in our memory might be in Map Frame, but Local Costmap is in Odom Frame
      // So we need to transform each time before drawing, so obstacles stay fixed in place and don't drift with Odom
      geometry_msgs::msg::PoseStamped pose_in, pose_out;
      pose_in.header.frame_id = "map"; // Assume SensorSim sends in map frame
      pose_in.header.stamp = rclcpp::Time(0); // Get latest transform
      pose_in.pose.position.x = pt_map.x;
      pose_in.pose.position.y = pt_map.y;
      pose_in.pose.position.z = pt_map.z;
      pose_in.pose.orientation.w = 1.0;

      try {
          if (tf) {
              tf->transform(pose_in, pose_out, costmap_frame, tf2::durationFromSec(0.1));
          } else {
              continue;
          }
      } catch (const tf2::TransformException & ex) {
          continue; 
      }

      // 2. Draw circle (using transformed pose_out)
      double center_x = pose_out.pose.position.x;
      double center_y = pose_out.pose.position.y;
      
      // Calculate grid range
    //   int min_mx, min_my, max_mx, max_my;
      double min_wx = center_x - inflation_radius_;
      double max_wx = center_x + inflation_radius_;
      double min_wy = center_y - inflation_radius_;
      double max_wy = center_y + inflation_radius_;

      unsigned int mx_start, my_start, mx_end, my_end;
      if (!master_grid.worldToMap(min_wx, min_wy, mx_start, my_start)) mx_start = my_start = 0;
      if (!master_grid.worldToMap(max_wx, max_wy, mx_end, my_end)) {
          mx_end = master_grid.getSizeInCellsX();
          my_end = master_grid.getSizeInCellsY();
      }
      
      // Safe boundaries
      mx_start = std::max(0u, mx_start);
      my_start = std::max(0u, my_start);
      mx_end = std::min((unsigned int)master_grid.getSizeInCellsX(), mx_end);
      my_end = std::min((unsigned int)master_grid.getSizeInCellsY(), my_end);

      // Fill
    //   for (unsigned int my = my_start; my < my_end; ++my) {
    //       for (unsigned int mx = mx_start; mx < mx_end; ++mx) {
    //           double cell_wx, cell_wy;
    //           master_grid.mapToWorld(mx, my, cell_wx, cell_wy);
    //           double dist = std::hypot(cell_wx - center_x, cell_wy - center_y);

    //           if (dist <= obstacle_radius_) {
    //               master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    //           }
    //       }
    //   }
      for (unsigned int my = my_start; my < my_end; ++my) {
          for (unsigned int mx = mx_start; mx < mx_end; ++mx) {
              
              double cell_wx, cell_wy;
              master_grid.mapToWorld(mx, my, cell_wx, cell_wy);
              double dist = std::hypot(cell_wx - center_x, cell_wy - center_y);

              // 1. Lethal zone
              if (dist <= obstacle_radius_) {
                  master_grid.setCost(mx, my, LETHAL_OBSTACLE);
              }
              // 2. Inflation gradient zone
              else if (dist <= inflation_radius_) {
                  // ROS Nav2 inflation formula: cost = exp(-factor * (dist - inscribed_radius)) * 253
                  double factor = std::exp(-1.0 * cost_scaling_factor_ * (dist - obstacle_radius_));
                  unsigned char cost = (unsigned char)(253 * factor); // 253 is INSCRIBED_INFLATED_OBSTACLE

                  // Ensure value is at least 1 (except FREE_SPACE=0)
                  if(cost < 1) cost = 1;

                  // === Key: Take maximum value (Max) ===
                  // If this cell is already a wall (LETHAL), we can't change it to only 50 Cost
                  // So we need to compare with the original value
                  unsigned char old_cost = master_grid.getCost(mx, my);
                  if (old_cost == nav2_costmap_2d::NO_INFORMATION) {
                      master_grid.setCost(mx, my, cost);
                  } else {
                      master_grid.setCost(mx, my, std::max(old_cost, cost));
                  }
              }
          }
      }
  }
}

}  // namespace sensor_layer