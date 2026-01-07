// #include "sensor_layer/sensor_layer.hpp"
// #include "pluginlib/class_list_macros.hpp"
// #include "geometry_msgs/msg/pose_array.hpp"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// PLUGINLIB_EXPORT_CLASS(Sensor_costmap_plugin::SensorLayer, nav2_costmap_2d::Layer)

// using nav2_costmap_2d::LETHAL_OBSTACLE;

// namespace Sensor_costmap_plugin
// {

// SensorLayer::SensorLayer() {}
// SensorLayer::~SensorLayer() {}

// void SensorLayer::onInitialize()
// {
//   current_ = true;
//   auto node = node_.lock(); 
//   if (!node) {
//     throw std::runtime_error{"Failed to lock node"};
//   }

//   sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
//     "/sensors/detected_obstacles", 10,
//     std::bind(&SensorLayer::poseArrayCallback, this, std::placeholders::_1));
    
//   RCLCPP_INFO(node->get_logger(), "SensorLayer (Radius Mode) initialized!");
// }

// void SensorLayer::poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
// {
//   std::lock_guard<std::mutex> lock(data_mutex_);
//   observation_buffer_.clear();

//   std::string costmap_frame = layered_costmap_->getGlobalFrameID();
  
//   // === 修正 1: 直接使用 tf_ ===
//   auto tf = tf_;

//   for (const auto& pose : msg->poses)
//   {
//       geometry_msgs::msg::PoseStamped pose_in, pose_out;
//       pose_in.header = msg->header;
//       pose_in.pose = pose;

//       try {
//           if (tf) {
//               tf->transform(pose_in, pose_out, costmap_frame, tf2::durationFromSec(0.1));
              
//               geometry_msgs::msg::Point pt;
//               pt.x = pose_out.pose.position.x;
//               pt.y = pose_out.pose.position.y;
//               pt.z = pose_out.pose.position.z;
//               observation_buffer_.push_back(pt);
//           }
//       } catch (const tf2::TransformException & ex) {
//           continue;
//       }
//   }
// }

// void SensorLayer::updateBounds(
//   double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
//   double * min_x, double * min_y, double * max_x, double * max_y)
// {
//   if (!enabled_) return;
//   std::lock_guard<std::mutex> lock(data_mutex_);
  
//   double radius = 0.5; 
//   double margin = 0.1; 

//   for (const auto & pt : observation_buffer_)
//   {
//     *min_x = std::min(*min_x, pt.x - radius - margin);
//     *min_y = std::min(*min_y, pt.y - radius - margin);
//     *max_x = std::max(*max_x, pt.x + radius + margin);
//     *max_y = std::max(*max_y, pt.y + radius + margin);
//   }
// }

// void SensorLayer::updateCosts(
//   nav2_costmap_2d::Costmap2D & master_grid,
//   int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
// {
//   if (!enabled_) return;
//   std::lock_guard<std::mutex> lock(data_mutex_);

//   double obstacle_radius = 0.5; 
//   double resolution = master_grid.getResolution();
//   int radius_cells = std::ceil(obstacle_radius / resolution);

//   for (const auto & pt : observation_buffer_)
//   {
//     unsigned int mx, my;
//     if (master_grid.worldToMap(pt.x, pt.y, mx, my))
//     {
//       for (int dx = -radius_cells; dx <= radius_cells; dx++)
//       {
//         for (int dy = -radius_cells; dy <= radius_cells; dy++)
//         {
//           if (dx*dx + dy*dy <= radius_cells*radius_cells)
//           {
//             // === 修正 2: 使用 int 來計算，避免 unsigned 的邏輯警告 ===
//             int nx = (int)mx + dx;
//             int ny = (int)my + dy;
            
//             if (nx >= 0 && ny >= 0 && 
//                 nx < (int)master_grid.getSizeInCellsX() && 
//                 ny < (int)master_grid.getSizeInCellsY())
//             {
//               master_grid.setCost(nx, ny, LETHAL_OBSTACLE);
//             }
//           }
//         }
//       }
//     }
//   }
// }

// }  // namespace sensor_layer



// #include "sensor_layer/sensor_layer.hpp"
// #include "pluginlib/class_list_macros.hpp"
// #include "geometry_msgs/msg/pose_array.hpp"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include <cmath> // 用於 pow, sqrt

// PLUGINLIB_EXPORT_CLASS(Sensor_costmap_plugin::SensorLayer, nav2_costmap_2d::Layer)

// using nav2_costmap_2d::LETHAL_OBSTACLE;
// using nav2_costmap_2d::NO_INFORMATION;

// namespace Sensor_costmap_plugin
// {

// SensorLayer::SensorLayer() {}
// SensorLayer::~SensorLayer() {}

// void SensorLayer::onInitialize()
// {
//   current_ = true;
//   enabled_ = true;
  
//   auto node = node_.lock(); 
//   if (!node) {
//     throw std::runtime_error{"Failed to lock node"};
//   }

//   // 1. 參數設定 (模仿 ObjectLayer)
//   // 你可以在 nav2_params.yaml 裡調整這些數值
//   node->declare_parameter(name_ + ".obstacle_radius", 0.1); 
//   node->declare_parameter(name_ + ".inflation_radius", 0.2); // 讓障礙物稍微胖一點
  
//   node->get_parameter(name_ + ".obstacle_radius", obstacle_radius_);
//   node->get_parameter(name_ + ".inflation_radius", inflation_radius_);

//   // 2. 訂閱 PoseArray
//   sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
//     "/sensors/detected_obstacles", 10,
//     std::bind(&SensorLayer::poseArrayCallback, this, std::placeholders::_1));
    
//   RCLCPP_INFO(node->get_logger(), "SensorLayer Initialized! Radius: %.2f", obstacle_radius_);
// }

// void SensorLayer::poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
// {
//   std::lock_guard<std::mutex> lock(data_mutex_);
//   // 這裡只負責接收資料，TF 轉換留到 updateCosts 做 (比較安全，因為時間戳會變)
//   latest_pose_array_ = *msg;
//   has_new_data_ = true;
// }

// void SensorLayer::updateBounds(
//   double robot_x, double robot_y, double robot_yaw,
//   double * min_x, double * min_y, double * max_x, double * max_y)
// {
//   if (!enabled_ || !has_new_data_) return;
//   std::lock_guard<std::mutex> lock(data_mutex_);
  
//   // 這裡我們做一個比較寬鬆的範圍更新，確保系統會去重繪
//   // 為了保險，我們把每個點都納入範圍
//   for(const auto& pose : latest_pose_array_.poses) {
//       // 這裡先簡單用 raw data 更新邊界，精確的位置在 updateCosts 算
//       // 注意：這裡假設 Pose 是在 map frame，可能會跟 local costmap (odom) 有偏差
//       // 但只要範圍夠大通常沒問題
//       *min_x = std::min(*min_x, pose.position.x - inflation_radius_ - 1.0);
//       *min_y = std::min(*min_y, pose.position.y - inflation_radius_ - 1.0);
//       *max_x = std::max(*max_x, pose.position.x + inflation_radius_ + 1.0);
//       *max_y = std::max(*max_y, pose.position.y + inflation_radius_ + 1.0);
//   }
// }

// void SensorLayer::updateCosts(
//   nav2_costmap_2d::Costmap2D & master_grid,
//   int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
// {
//   if (!enabled_ || !has_new_data_) return;
//   std::lock_guard<std::mutex> lock(data_mutex_);

//   // 取得 Costmap 的 Frame (通常是 odom)
//   std::string costmap_frame = layered_costmap_->getGlobalFrameID();
//   auto tf = tf_;

//   // 模仿 ObjectLayer: 遍歷所有障礙物
//   for (const auto& raw_pose : latest_pose_array_.poses)
//   {
//       geometry_msgs::msg::PoseStamped pose_in, pose_out;
//       pose_in.header = latest_pose_array_.header; // 重要！使用來源的 frame (map)
//       pose_in.pose = raw_pose;

//       // === 1. TF 轉換 (模仿 eliminateObject 的邏輯) ===
//       try {
//           if (!tf) continue;

//           // 確保 frame_id 不為空
//           if(pose_in.header.frame_id.empty()) {
//              pose_in.header.frame_id = "map";
//           }

//           // 轉到 Costmap Frame (odom)
//           tf->transform(pose_in, pose_out, costmap_frame, tf2::durationFromSec(0.1));

//       } catch (const tf2::TransformException & ex) {
//           RCLCPP_WARN_THROTTLE(node_.lock()->get_logger(), *node_.lock()->get_clock(), 2000, 
//               "SensorLayer TF Error: %s", ex.what());
//           continue;
//       }

//       // === 2. 畫圓 (模仿 ExpandPointWithCircle) ===
//       // 使用轉換後的座標 pose_out
//       double center_x = pose_out.pose.position.x;
//       double center_y = pose_out.pose.position.y;
      
//       // 計算要畫的範圍 (Bounding Box)
//       double min_world_x = center_x - inflation_radius_;
//       double max_world_x = center_x + inflation_radius_;
//       double min_world_y = center_y - inflation_radius_;
//       double max_world_y = center_y + inflation_radius_;

//       // 將世界座標轉為柵格座標 (Grid Index)
//       unsigned int min_mx, min_my, max_mx, max_my;
//       if (!master_grid.worldToMap(min_world_x, min_world_y, min_mx, min_my)) min_mx = min_my = 0;
//       if (!master_grid.worldToMap(max_world_x, max_world_y, max_mx, max_my)) {
//            max_mx = master_grid.getSizeInCellsX();
//            max_my = master_grid.getSizeInCellsY();
//       }

//       // 限制在邊界內
//       min_mx = std::max(0u, min_mx);
//       min_my = std::max(0u, min_my);
//       max_mx = std::min((unsigned int)master_grid.getSizeInCellsX(), max_mx);
//       max_my = std::min((unsigned int)master_grid.getSizeInCellsY(), max_my);

//       double resolution = master_grid.getResolution();

//       // 遍歷 Bounding Box 內的每一格
//       for (unsigned int my = min_my; my < max_my; ++my) {
//           for (unsigned int mx = min_mx; mx < max_mx; ++mx) {
              
//               // 轉回世界座標算距離
//               double cell_wx, cell_wy;
//               master_grid.mapToWorld(mx, my, cell_wx, cell_wy);

//               double dist = std::hypot(cell_wx - center_x, cell_wy - center_y);

//               // 如果在圓內
//               if (dist <= obstacle_radius_) {
//                   // 直接設為致命障礙物
//                   master_grid.setCost(mx, my, LETHAL_OBSTACLE);
//               }
//               // 如果在膨脹圈內 (可選)
//               else if (dist <= inflation_radius_) {
//                    // 這裡可以模仿 ObjectLayer 做漸層，但為了單純先畫實心
//                    // 或者你要畫小一點的 cost:
//                    // master_grid.setCost(mx, my, 128); 
//               }
//           }
//       }
//   }
// }

// }  // namespace sensor_layer





// #include "sensor_layer/sensor_layer.hpp"
// #include "pluginlib/class_list_macros.hpp"
// #include "geometry_msgs/msg/pose_array.hpp"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include <cmath> 

// // 註冊 Plugin (注意 namespace)
// PLUGINLIB_EXPORT_CLASS(Sensor_costmap_plugin::SensorLayer, nav2_costmap_2d::Layer)

// using nav2_costmap_2d::LETHAL_OBSTACLE;
// using nav2_costmap_2d::NO_INFORMATION;

// namespace Sensor_costmap_plugin
// {

// SensorLayer::SensorLayer() {}
// SensorLayer::~SensorLayer() {}

// void SensorLayer::onInitialize()
// {
//   current_ = true;
//   enabled_ = true;
  
//   auto node = node_.lock(); 
//   if (!node) {
//     throw std::runtime_error{"Failed to lock node"};
//   }

//   // 參數設定：這些可以在 nav2_params.yaml 裡面改
//   node->declare_parameter(name_ + ".obstacle_radius", 0.15); // 畫出來的點多大
//   node->declare_parameter(name_ + ".inflation_radius", 0.2); // (選項)
  
//   node->get_parameter(name_ + ".obstacle_radius", obstacle_radius_);
//   node->get_parameter(name_ + ".inflation_radius", inflation_radius_);

//   // 訂閱 SensorSim 發出的 PoseArray
//   sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
//     "/sensors/detected_obstacles", 10,
//     std::bind(&SensorLayer::poseArrayCallback, this, std::placeholders::_1));
    
//   RCLCPP_INFO(node->get_logger(), "SensorLayer Initialized! Draw Radius: %.2f", obstacle_radius_);
// }

// void SensorLayer::poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
// {
//   std::lock_guard<std::mutex> lock(data_mutex_);
//   latest_pose_array_ = *msg;
//   has_new_data_ = true;
// }

// void SensorLayer::updateBounds(
//   double robot_x, double robot_y, double robot_yaw,
//   double * min_x, double * min_y, double * max_x, double * max_y)
// {
//   if (!enabled_ || !has_new_data_) return;
//   std::lock_guard<std::mutex> lock(data_mutex_);
  
//   // 擴大更新範圍，確保 Costmap 會重繪這些區域
//   for(const auto& pose : latest_pose_array_.poses) {
//       // 在障礙物點周圍預留足夠的空間
//       double range = obstacle_radius_ + 1.0; 
//       *min_x = std::min(*min_x, pose.position.x - range);
//       *min_y = std::min(*min_y, pose.position.y - range);
//       *max_x = std::max(*max_x, pose.position.x + range);
//       *max_y = std::max(*max_y, pose.position.y + range);
//   }
// }

// void SensorLayer::updateCosts(
//   nav2_costmap_2d::Costmap2D & master_grid,
//   int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
// {
//   if (!enabled_ || !has_new_data_) return;
//   std::lock_guard<std::mutex> lock(data_mutex_);

//   std::string costmap_frame = layered_costmap_->getGlobalFrameID();
//   auto tf = tf_;

//   // 遍歷所有偵測到的障礙點
//   for (const auto& raw_pose : latest_pose_array_.poses)
//   {
//       geometry_msgs::msg::PoseStamped pose_in, pose_out;
//       pose_in.header = latest_pose_array_.header; 
//       pose_in.pose = raw_pose;

//       // 1. TF 轉換：確保座標正確對應到 Costmap (odom/map)
//       try {
//           if (!tf) continue;
//           if(pose_in.header.frame_id.empty()) pose_in.header.frame_id = "map";

//           tf->transform(pose_in, pose_out, costmap_frame, tf2::durationFromSec(0.1));
//       } catch (const tf2::TransformException & ex) {
//           continue; 
//       }

//       // 2. 畫圓 (在接觸點畫上 obstacle_radius 大小的實心圓)
//       double center_x = pose_out.pose.position.x;
//       double center_y = pose_out.pose.position.y;
      
//       // 計算格子範圍
//       int min_mx, min_my, max_mx, max_my;
//       double min_wx = center_x - obstacle_radius_;
//       double max_wx = center_x + obstacle_radius_;
//       double min_wy = center_y - obstacle_radius_;
//       double max_wy = center_y + obstacle_radius_;

//       // 世界座標 -> 柵格座標
//       unsigned int mx_start, my_start, mx_end, my_end;
//       if (!master_grid.worldToMap(min_wx, min_wy, mx_start, my_start)) mx_start = my_start = 0;
//       if (!master_grid.worldToMap(max_wx, max_wy, mx_end, my_end)) {
//           mx_end = master_grid.getSizeInCellsX();
//           my_end = master_grid.getSizeInCellsY();
//       }
      
//       // 安全邊界檢查
//       mx_start = std::max(0u, mx_start);
//       my_start = std::max(0u, my_start);
//       mx_end = std::min((unsigned int)master_grid.getSizeInCellsX(), mx_end);
//       my_end = std::min((unsigned int)master_grid.getSizeInCellsY(), my_end);

//       // 填滿圓形區域
//       for (unsigned int my = my_start; my < my_end; ++my) {
//           for (unsigned int mx = mx_start; mx < mx_end; ++mx) {
//               double cell_wx, cell_wy;
//               master_grid.mapToWorld(mx, my, cell_wx, cell_wy);
//               double dist = std::hypot(cell_wx - center_x, cell_wy - center_y);

//               if (dist <= obstacle_radius_) {
//                   master_grid.setCost(mx, my, LETHAL_OBSTACLE);
//               }
//           }
//       }
//   }
// }

// }  // namespace sensor_layer



#include "sensor_layer/sensor_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath> 

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
  
  node->get_parameter(name_ + ".obstacle_radius", obstacle_radius_);
  node->get_parameter(name_ + ".inflation_radius", inflation_radius_);

  sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
    "/sensors/detected_obstacles", 10,
    std::bind(&SensorLayer::poseArrayCallback, this, std::placeholders::_1));
    
  RCLCPP_INFO(node->get_logger(), "SensorLayer (Persistent Mode) Initialized!");
}

// 當有人呼叫 /clear_costmaps 時會觸發這個
void SensorLayer::reset()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    persistent_obstacles_.clear();
    current_ = false;
}

void SensorLayer::poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // 取得 Costmap 的 Frame (通常是 odom)
  // 如果 SensorSim 發的是 map，我們要把點存成 map 還是 odom?
  // 建議：為了讓障礙物「固定在世界上」，這裡我們假設 SensorSim 發送的是 "map" frame 的座標
  // 我們直接存下來即可。
  
  for(const auto& new_pose : msg->poses)
  {
      bool is_duplicate = false;
      
      // === 簡單的空間過濾 ===
      // 檢查這個新點是否已經存在於我們的記憶庫中 (距離太近就算重複)
      for(const auto& existing_pt : persistent_obstacles_)
      {
          double dist = std::hypot(new_pose.position.x - existing_pt.x, 
                                   new_pose.position.y - existing_pt.y);
          // 如果新點跟舊點距離小於 7.5公分，就不存了，節省效能
          if(dist < 0.075) {
              is_duplicate = true;
              break;
          }
      }

      // 如果是新的點，就存進去
      if(!is_duplicate) {
          geometry_msgs::msg::Point pt;
          pt.x = new_pose.position.x;
          pt.y = new_pose.position.y;
          pt.z = new_pose.position.z;
          persistent_obstacles_.push_back(pt);
      }
  }
}

void SensorLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) return;
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // 遍歷「記憶庫」中的所有點來更新邊界
  double range = obstacle_radius_ + 0.05; // 多一點 buffer

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

  // 遍歷「記憶庫」中的所有障礙點
  for (const auto& pt_map : persistent_obstacles_)
  {
      // 1. TF 轉換
      // 因為我們記憶庫裡的點可能是 Map Frame，但 Local Costmap 是 Odom Frame
      // 所以每次畫圖前都要轉一次，這樣障礙物才會固定在原地，不會跟著 Odom 飄移
      geometry_msgs::msg::PoseStamped pose_in, pose_out;
      pose_in.header.frame_id = "map"; // 假設 SensorSim 發的是 map
      pose_in.header.stamp = rclcpp::Time(0); // 拿最新的 transform
      pose_in.pose.position = pt_map;
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

      // 2. 畫圓 (使用轉換後的 pose_out)
      double center_x = pose_out.pose.position.x;
      double center_y = pose_out.pose.position.y;
      
      // 計算格子範圍
      int min_mx, min_my, max_mx, max_my;
      double min_wx = center_x - obstacle_radius_;
      double max_wx = center_x + obstacle_radius_;
      double min_wy = center_y - obstacle_radius_;
      double max_wy = center_y + obstacle_radius_;

      unsigned int mx_start, my_start, mx_end, my_end;
      if (!master_grid.worldToMap(min_wx, min_wy, mx_start, my_start)) mx_start = my_start = 0;
      if (!master_grid.worldToMap(max_wx, max_wy, mx_end, my_end)) {
          mx_end = master_grid.getSizeInCellsX();
          my_end = master_grid.getSizeInCellsY();
      }
      
      // 安全邊界
      mx_start = std::max(0u, mx_start);
      my_start = std::max(0u, my_start);
      mx_end = std::min((unsigned int)master_grid.getSizeInCellsX(), mx_end);
      my_end = std::min((unsigned int)master_grid.getSizeInCellsY(), my_end);

      // 填滿
      for (unsigned int my = my_start; my < my_end; ++my) {
          for (unsigned int mx = mx_start; mx < mx_end; ++mx) {
              double cell_wx, cell_wy;
              master_grid.mapToWorld(mx, my, cell_wx, cell_wy);
              double dist = std::hypot(cell_wx - center_x, cell_wy - center_y);

              if (dist <= obstacle_radius_) {
                  master_grid.setCost(mx, my, LETHAL_OBSTACLE);
              }
          }
      }
  }
}

}  // namespace sensor_layer