// #ifndef SENSOR_LAYER_HPP_
// #define SENSOR_LAYER_HPP_

// #include "rclcpp/rclcpp.hpp"
// #include "nav2_costmap_2d/layer.hpp"
// #include "nav2_costmap_2d/layered_costmap.hpp"
// #include "sensor_msgs/msg/point_cloud2.hpp"
// #include "geometry_msgs/msg/pose_array.hpp"
// #include "tf2_ros/buffer.h"

// namespace Sensor_costmap_plugin
// {

// class SensorLayer : public nav2_costmap_2d::Layer
// {
// public:
//   SensorLayer();
//   virtual ~SensorLayer();

//   // 1. 初始化 (讀取參數)
//   virtual void onInitialize();

//   // 2. 更新邊界 (告訴 Costmap 我要在哪個範圍內畫畫)
//   virtual void updateBounds(
//     double robot_x, double robot_y, double robot_yaw,
//     double * min_x, double * min_y, double * max_x, double * max_y);

//   // 3. 更新數值 (真的把障礙物畫上去)
//   virtual void updateCosts(
//     nav2_costmap_2d::Costmap2D & master_grid,
//     int min_i, int min_j, int max_i, int max_j);

//   // 4. 重置
//   virtual void reset()
//   {
//     return;
//   }

//   // 5. 判斷是否清除 (這層通常只負責標記)
//   virtual bool isClearable() {return false;}

// private:
//   // 接收模擬感測器的 Callback
//   void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

//   rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
//   std::vector<geometry_msgs::msg::Point> observation_buffer_; // 暫存接收到的點
//   std::mutex data_mutex_;
// };

// }  // namespace Sensor_costmap_plugin
// #endif  // SENSOR_LAYER_HPP_


// #ifndef SENSOR_LAYER_HPP_
// #define SENSOR_LAYER_HPP_

// #include "rclcpp/rclcpp.hpp"
// #include "nav2_costmap_2d/layer.hpp"
// #include "nav2_costmap_2d/layered_costmap.hpp"
// #include "geometry_msgs/msg/pose_array.hpp"

// namespace Sensor_costmap_plugin
// {

// class SensorLayer : public nav2_costmap_2d::Layer
// {
// public:
//   SensorLayer();
//   virtual ~SensorLayer();

//   virtual void onInitialize();
//   virtual void updateBounds(
//     double robot_x, double robot_y, double robot_yaw,
//     double * min_x, double * min_y, double * max_x, double * max_y);

//   virtual void updateCosts(
//     nav2_costmap_2d::Costmap2D & master_grid,
//     int min_i, int min_j, int max_i, int max_j);

//   virtual void reset() { return; }
//   virtual bool isClearable() { return false; }

// private:
//   void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

//   rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
//   geometry_msgs::msg::PoseArray latest_pose_array_;
//   bool has_new_data_ = false;
//   std::mutex data_mutex_;

//   double obstacle_radius_ = 0.5;
//   double inflation_radius_ = 0.5;
// };

// }  // namespace sensor_layer

// #endif





#ifndef SENSOR_LAYER_HPP_
#define SENSOR_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector> // 需要 vector

namespace Sensor_costmap_plugin
{

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

  // 當 Costmap 被重置時 (例如呼叫 clear_costmaps service)，清空記憶
  virtual void reset();
  virtual bool isClearable() { return true; }

private:
  void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
  
  // === 關鍵修改：這就是我們的「記憶庫」 ===
  // 存放所有歷史偵測到的障礙物中心點 (World Frame / Map Frame)
  std::vector<geometry_msgs::msg::Point> persistent_obstacles_;
  
  std::mutex data_mutex_;

  double obstacle_radius_;
  double inflation_radius_;
};

}  // namespace sensor_layer

#endif