// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/pose_array.hpp"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"
// #include <cmath>
// #include <vector>

// class SensorSim : public rclcpp::Node
// {
// public:
//     SensorSim() : Node("sensor_sim_node")
//     {
//         // 發布 PoseArray (代表感測器打到的障礙物點)
//         publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/sensors/detected_obstacles", 10);
        
//         tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
//         tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//         // 10Hz 更新頻率
//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(100), 
//             std::bind(&SensorSim::timer_callback, this));
            
//         RCLCPP_INFO(this->get_logger(), "Sensor Simulation (3-Sensor Raycast) Started");
//     }

// private:
//     void timer_callback()
//     {
//         // 1. 取得機器人位置與姿態
//         double robot_x = 0.0;
//         double robot_y = 0.0;
//         double robot_yaw = 0.0;

//         try {
//             geometry_msgs::msg::TransformStamped t;
//             if (tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero)) {
//                 t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
//             } else {
//                 return; // TF 還沒準備好
//             }
//             robot_x = t.transform.translation.x;
//             robot_y = t.transform.translation.y;
            
//             // 計算 Yaw (方向)
//             double qx = t.transform.rotation.x;
//             double qy = t.transform.rotation.y;
//             double qz = t.transform.rotation.z;
//             double qw = t.transform.rotation.w;
//             robot_yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
            
//         } catch (const tf2::TransformException & ex) {
//             return;
//         }

//         // ==========================================
//         // 2. 場景設定 (虛擬障礙物位置)
//         // ==========================================
//         double obs_x = 1.5;      // 障礙物中心 X
//         double obs_y = 0.6;      // 障礙物中心 Y
//         double obs_radius = 0.3; // 障礙物半徑 (實體大小)
        
//         // 感測器設定
//         double max_range = 0.4;  // 感測器最遠能看到多遠
        
//         // 定義三個感測器在機器人身上的相對位置 (x, y)
//         struct Sensor { std::string name; double x; double y; };
//         std::vector<Sensor> sensors = {
//             {"Left",   0.075,  0.075}, // 左前方
//             {"Center", 0.075,  0.0}, // 正前方
//             {"Right",  0.075, -0.075}  // 右前方
//         };

//         // 準備要發送的 PoseArray
//         geometry_msgs::msg::PoseArray msg;
//         msg.header.stamp = this->now();
//         msg.header.frame_id = "map"; 

//         bool any_hit = false;

//         for (const auto& sensor : sensors)
//         {
//             // A. 計算感測器在世界座標的位置
//             double sensor_global_x = robot_x + sensor.x * cos(robot_yaw) - sensor.y * sin(robot_yaw);
//             double sensor_global_y = robot_y + sensor.x * sin(robot_yaw) + sensor.y * cos(robot_yaw);

//             // B. 射線方向 (假設感測器朝車頭前方)
//             double dir_x = cos(robot_yaw);
//             double dir_y = sin(robot_yaw);

//             // C. 數學運算：射線與圓的交點 (Ray-Circle Intersection)
//             // L = CircleCenter - SensorPos
//             double Lx = obs_x - sensor_global_x;
//             double Ly = obs_y - sensor_global_y;

//             // t_ca = L dot Direction
//             double t_ca = Lx * dir_x + Ly * dir_y;

//             if (t_ca < 0) continue; // 障礙物在背後

//             double d2 = (Lx * Lx + Ly * Ly) - (t_ca * t_ca); // 垂直距離平方
//             double r2 = obs_radius * obs_radius;

//             if (d2 > r2) continue; // 射線沒射中圓柱體

//             double t_hc = sqrt(r2 - d2);
//             double dist_to_surface = t_ca - t_hc; // 距離表面的距離

//             // D. 判斷是否在偵測範圍內
//             // if (dist_to_surface > 0 && dist_to_surface < max_range)
//             if (dist_to_surface > 0 && dist_to_surface < max_range)
//             {
//                 // 算出「接觸點」的世界座標
//                 double hit_x = sensor_global_x + dist_to_surface * dir_x;
//                 double hit_y = sensor_global_y + dist_to_surface * dir_y;

//                 geometry_msgs::msg::Pose p;
//                 p.position.x = hit_x;
//                 p.position.y = hit_y;
//                 p.position.z = 0.0;
//                 p.orientation.w = 1.0;
//                 msg.poses.push_back(p);
                
//                 any_hit = true;
                
//                 // Debug Log
//                 RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
//                     "[%s] Detected at dist: %.2fm", sensor.name.c_str(), dist_to_surface);
//             }
//         }

//         // 只有偵測到才發送，沒偵測到就不發 (Layer 不會清除舊的，除非你有設定清除機制)
//         // 為了讓 Layer 能更新(清除)，通常建議發送空陣列代表「現在沒看到東西」
//         // 但如果你的邏輯是「只要看到就加上去」，可以一直發。
//         // 這裡我們選擇：總是發布 msg (即使是空的)，這樣 Layer 才知道現在狀況
//         publisher_->publish(msg);
//     }

//     rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
//     std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
//     std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<SensorSim>());
//     rclcpp::shutdown();
//     return 0;
// }





#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <cmath>
#include <vector>
#include <string>

// 用來轉換角度 (度 -> 弧度)
inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

class SensorSim : public rclcpp::Node
{
public:
    SensorSim() : Node("sensor_sim_node")
    {
        // 發布 PoseArray (代表感測器打到的障礙物點)
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/sensors/detected_obstacles", 10);
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 10Hz 更新頻率 (模擬感測器刷新率)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&SensorSim::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Sensor Simulation (Angled VL53) Started");
    }

private:
    void timer_callback()
    {
        // ==========================================
        // 1. 取得機器人位置與姿態 (Robot State)
        // ==========================================
        double robot_x = 0.0;
        double robot_y = 0.0;
        double robot_yaw = 0.0;

        try {
            geometry_msgs::msg::TransformStamped t;
            if (tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero)) {
                t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            } else {
                return; // TF 還沒準備好，跳過這次迴圈
            }
            robot_x = t.transform.translation.x;
            robot_y = t.transform.translation.y;
            
            // 計算 Yaw (機器人面向)
            double qx = t.transform.rotation.x;
            double qy = t.transform.rotation.y;
            double qz = t.transform.rotation.z;
            double qw = t.transform.rotation.w;
            robot_yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
            
        } catch (const tf2::TransformException & ex) {
            return;
        }

        // ==========================================
        // 2. 模擬環境設定 (Virtual Obstacle)
        // ==========================================
        // 在地圖上放一個圓形障礙物
        double obs_x = 1.5;      // 障礙物中心 X
        double obs_y = 0.5;      // 障礙物中心 Y (稍微偏左一點，讓左邊感測器容易看到)
        double obs_radius = 0.3; // 障礙物半徑 (類似一個大柱子)
        
        // 感測器參數 (VL53L1X 約 2-4m，VL53L0X 約 1-2m，這裡設 1.5m)
        double max_range = 0.5; 

        // ==========================================
        // 3. 感測器配置 (關鍵修改處)
        // ==========================================
        struct Sensor { 
            std::string name; 
            double x;    // 安裝位置 X (相對於 base_link)
            double y;    // 安裝位置 Y (相對於 base_link)
            double yaw;  // 安裝角度 (相對於 base_link)
        };

        std::vector<Sensor> sensors = {
            // 左感測器：安裝在左前方，朝向左邊 45 度
            {"Left",   0.075,  0.075, deg2rad(45.0)}, 
            
            // 中感測器：安裝在正前方，朝向 0 度
            {"Center", 0.075,  0.0,   deg2rad(0.0)}, 
            
            // 右感測器：安裝在右前方，朝向右邊 -45 度
            {"Right",  0.075, -0.075, deg2rad(-45.0)}
        };

        // 準備 PoseArray 訊息
        geometry_msgs::msg::PoseArray msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map"; // 我們算出來的是世界座標，直接送 map frame

        bool any_hit = false;

        for (const auto& sensor : sensors)
        {
            // A. 計算感測器「本體」在世界座標的位置
            // 剛體變換矩陣運算
            double sensor_global_x = robot_x + sensor.x * cos(robot_yaw) - sensor.y * sin(robot_yaw);
            double sensor_global_y = robot_y + sensor.x * sin(robot_yaw) + sensor.y * cos(robot_yaw);

            // B. 計算感測器「射線方向」 (關鍵修改！)
            // 射線角度 = 機器人角度 + 感測器安裝角度
            double global_sensor_angle = robot_yaw + sensor.yaw;
            double dir_x = cos(global_sensor_angle);
            double dir_y = sin(global_sensor_angle);

            // 
            // C. 數學運算：射線與圓的交點 (Ray-Circle Intersection)
            // L = CircleCenter - SensorPos (從感測器指向圓心的向量)
            double Lx = obs_x - sensor_global_x;
            double Ly = obs_y - sensor_global_y;

            // t_ca = L dot Direction (圓心投影在射線上的距離)
            double t_ca = Lx * dir_x + Ly * dir_y;

            if (t_ca < 0) continue; // 障礙物在感測器背後

            double d2 = (Lx * Lx + Ly * Ly) - (t_ca * t_ca); // 圓心到射線的最短距離平方
            double r2 = obs_radius * obs_radius;

            if (d2 > r2) continue; // 射線完全沒碰到圓

            double t_hc = sqrt(r2 - d2); // 從切點到圓心的距離
            double dist_to_surface = t_ca - t_hc; // 最終距離：感測器到圓表面的距離

            // D. 判斷是否在有效偵測範圍內
            if (dist_to_surface > 0 && dist_to_surface < max_range)
            {
                // 計算「接觸點」的世界座標
                double hit_x = sensor_global_x + dist_to_surface * dir_x;
                double hit_y = sensor_global_y + dist_to_surface * dir_y;

                geometry_msgs::msg::Pose p;
                p.position.x = hit_x;
                p.position.y = hit_y;
                p.position.z = 0.0; // 地面上
                p.orientation.w = 1.0;
                msg.poses.push_back(p);
                
                any_hit = true;
                
                // Debug Log (用 Throttle 防止洗版)
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "HIT! [%s] Dist: %.2fm at (%.2f, %.2f)", 
                    sensor.name.c_str(), dist_to_surface, hit_x, hit_y);
            }
        }

        // 發布結果 (即使是空的也要發布，讓 SensorLayer 知道現在沒東西)
        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorSim>());
    rclcpp::shutdown();
    return 0;
}