// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/point_cloud2.hpp"
// #include "sensor_msgs/point_cloud2_iterator.hpp"
// #include "geometry_msgs/msg/point.hpp"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"
// #include <cmath> // 需要數學函式庫
// #include <vector>

// class SensorSim : public rclcpp::Node
// {
// public:
//     SensorSim() : Node("sensor_sim_node")
//     {
//         publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensors/merged_tof_points", 10);
        
//         tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
//         tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(50), 
//             std::bind(&SensorSim::timer_callback, this));
//     }

// private:
//     void timer_callback()
//     {
//         // 1. 取得機器人在 Map 上的位置與角度
//         double robot_x = 0.0;
//         double robot_y = 0.0;
//         double robot_yaw = 0.0;

//         try {
//             geometry_msgs::msg::TransformStamped t;
//             t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
//             robot_x = t.transform.translation.x;
//             robot_y = t.transform.translation.y;
            
//             // 計算 Yaw (角度)
//             double qx = t.transform.rotation.x;
//             double qy = t.transform.rotation.y;
//             double qz = t.transform.rotation.z;
//             double qw = t.transform.rotation.w;
//             robot_yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
//         } catch (const tf2::TransformException & ex) {
//             return;
//         }

//         // 2. 定義你的圓形障礙物 (世界座標)
//         double obs_x = 1.5;
//         double obs_y = 1.0;
//         double obs_radius = 0.5;

//         // 3. 定義三個感測器在機器人身上的相對位置 (x, y)
//         // 假設都朝正前方看
//         struct Sensor { double x; double y; };
//         std::vector<Sensor> sensors = {
//             {0.1,  0.1}, // 左 (左偏 10cm)
//             {0.1,  0.0}, // 中
//             {0.1, -0.1}  // 右 (右偏 10cm)
//         };

//         double max_range = 1.0; // 感測器極限 1 公尺
//         std::vector<geometry_msgs::msg::Point> hits;

//         for (const auto& sensor : sensors)
//         {
//             // 算出這個感測器現在在「世界座標」的哪裡
//             // 旋轉公式: 
//             // x' = x cos - y sin + robot_x
//             // y' = x sin + y cos + robot_y
//             double sensor_global_x = robot_x + sensor.x * cos(robot_yaw) - sensor.y * sin(robot_yaw);
//             double sensor_global_y = robot_y + sensor.x * sin(robot_yaw) + sensor.y * cos(robot_yaw);

//             // 算出感測器的「視線方向向量」 (假設感測器跟著機器人轉，朝正前方)
//             double dir_x = cos(robot_yaw);
//             double dir_y = sin(robot_yaw);

//             // === 數學魔法：計算射線(Ray)與圓(Circle)的交點 ===
//             // 向量 L = 圓心 - 感測器位置
//             double Lx = obs_x - sensor_global_x;
//             double Ly = obs_y - sensor_global_y;

//             // t_ca = L dot Direction (投影長度)
//             double t_ca = Lx * dir_x + Ly * dir_y;

//             if (t_ca < 0) continue; // 障礙物在背後，看不到

//             // d2 = L dot L - t_ca * t_ca (射線到圓心的垂直距離平方)
//             double d2 = (Lx * Lx + Ly * Ly) - (t_ca * t_ca);
//             double r2 = obs_radius * obs_radius;

//             if (d2 > r2) continue; // 射線沒射中圓，Miss

//             // t_hc = sqrt(r^2 - d^2) (從切點到穿出點的半長)
//             double t_hc = sqrt(r2 - d2);

//             // t0 = t_ca - t_hc (最近的交點距離)
//             double dist = t_ca - t_hc;

//             // 如果距離在感測範圍內 (0 ~ 1.0m)
//             if (dist > 0 && dist < max_range)
//             {
//                 // 這就是我們要的點！
//                 // 把這個距離換算回「相對於 robot_base」的座標
//                 // 簡單算法：感測器位置 + 距離 (因為是朝正前方)
//                 hits.push_back(create_point(sensor.x + dist, sensor.y, 0.1));
//             }
//         }

//         // 4. 發布
//         if (!hits.empty()) {
//             publish_points(hits);
//         } else {
//             publish_points({});
//         }
//     }

//     geometry_msgs::msg::Point create_point(double x, double y, double z) {
//         geometry_msgs::msg::Point p; p.x = x; p.y = y; p.z = z; return p;
//     }

//     void publish_points(const std::vector<geometry_msgs::msg::Point>& points)
//     {
//         auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
//         msg->header.stamp = this->now();
//         msg->header.frame_id = "base_link"; 
//         msg->height = 1; 
//         msg->width = points.size();
//         msg->is_dense = false;
//         msg->is_bigendian = false;

//         sensor_msgs::PointCloud2Modifier modifier(*msg);
//         modifier.setPointCloud2FieldsByString(1, "xyz");
//         modifier.resize(points.size());

//         sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
//         sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
//         sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

//         for (const auto& p : points) {
//             *iter_x = p.x; *iter_y = p.y; *iter_z = p.z;
//             ++iter_x; ++iter_y; ++iter_z;
//         }
//         publisher_->publish(std::move(msg));
//     }

//     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
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



// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/point_cloud2.hpp"
// #include "sensor_msgs/point_cloud2_iterator.hpp"
// #include "geometry_msgs/msg/point.hpp"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"
// #include <cmath>
// #include <vector>

// class SensorSim : public rclcpp::Node
// {
// public:
//     SensorSim() : Node("sensor_sim_node")
//     {
//         publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensors/merged_tof_points", 10);
        
//         tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
//         tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//         // 設定 10Hz (100ms) 就好，不用太快，方便看 log
//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(100), 
//             std::bind(&SensorSim::timer_callback, this));
            
//         RCLCPP_INFO(this->get_logger(), "Sensor Simulation Node Started (Debug Mode)");
//     }

// private:
//     void timer_callback()
//     {
//         double robot_x = 0.0;
//         double robot_y = 0.0;
//         double robot_yaw = 0.0;

//         try {
//             geometry_msgs::msg::TransformStamped t;
//             // 等待一下 TF，避免剛啟動時報錯
//             if (tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero)) {
//                 t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
//             } else {
//                 RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for TF map->base_link...");
//                 return;
//             }
            
//             robot_x = t.transform.translation.x;
//             robot_y = t.transform.translation.y;
            
//             double qx = t.transform.rotation.x;
//             double qy = t.transform.rotation.y;
//             double qz = t.transform.rotation.z;
//             double qw = t.transform.rotation.w;
//             robot_yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
            
//         } catch (const tf2::TransformException & ex) {
//             RCLCPP_ERROR(this->get_logger(), "TF Error: %s", ex.what());
//             return;
//         }

//         // 障礙物設定
//         double obs_x = 1.5;
//         double obs_y = 1.0;
//         double obs_radius = 0.5;

//         // 加大偵測距離到 2.5m (為了測試方便)
//         double max_range = 2.5; 

//         struct Sensor { std::string name; double x; double y; };
//         std::vector<Sensor> sensors = {
//             {"Left",   0.1,  0.1},
//             {"Center", 0.1,  0.0},
//             {"Right",  0.1, -0.1}
//         };

//         std::vector<geometry_msgs::msg::Point> hits;
//         bool any_hit = false;

//         for (const auto& sensor : sensors)
//         {
//             // 機器人目前的世界座標
//             double sensor_global_x = robot_x + sensor.x * cos(robot_yaw) - sensor.y * sin(robot_yaw);
//             double sensor_global_y = robot_y + sensor.x * sin(robot_yaw) + sensor.y * cos(robot_yaw);

//             // 射線方向 (假設跟著車頭轉)
//             double dir_x = cos(robot_yaw);
//             double dir_y = sin(robot_yaw);

//             // Ray-Circle Intersection Math
//             double Lx = obs_x - sensor_global_x;
//             double Ly = obs_y - sensor_global_y;
//             double t_ca = Lx * dir_x + Ly * dir_y;

//             if (t_ca < 0) continue; // 背對障礙物

//             double d2 = (Lx * Lx + Ly * Ly) - (t_ca * t_ca);
//             double r2 = obs_radius * obs_radius;

//             if (d2 > r2) continue; // 沒射中

//             double t_hc = sqrt(r2 - d2);
//             double dist = t_ca - t_hc;

//             if (dist > 0 && dist < max_range)
//             {
//                 any_hit = true;
//                 hits.push_back(create_point(sensor.x + dist, sensor.y, 0.1));
                
//                 // [DEBUG] 印出偵測到的距離
//                 RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
//                     "HIT! [%s] Dist: %.2f m | Robot at (%.2f, %.2f)", 
//                     sensor.name.c_str(), dist, robot_x, robot_y);
//             }
//         }

//         if (!hits.empty()) {
//             publish_points(hits);
//         } else {
//             publish_points({});
//             // [DEBUG] 如果太久沒偵測到，每 5 秒印一次目前位置，確認機器人是不是還活著
//             RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
//                 "No obstacle. Robot at (%.2f, %.2f) facing %.2f rad", robot_x, robot_y, robot_yaw);
//         }
//     }

//     geometry_msgs::msg::Point create_point(double x, double y, double z) {
//         geometry_msgs::msg::Point p; p.x = x; p.y = y; p.z = z; return p;
//     }

//     void publish_points(const std::vector<geometry_msgs::msg::Point>& points)
//     {
//         auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
//         msg->header.stamp = this->now();
//         msg->header.frame_id = "base_link"; 
//         msg->height = 1; 
//         msg->width = points.size();
//         msg->is_dense = false;
//         msg->is_bigendian = false;

//         sensor_msgs::PointCloud2Modifier modifier(*msg);
//         modifier.setPointCloud2FieldsByString(1, "xyz");
//         modifier.resize(points.size());

//         sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
//         sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
//         sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

//         for (const auto& p : points) {
//             *iter_x = p.x; *iter_y = p.y; *iter_z = p.z;
//             ++iter_x; ++iter_y; ++iter_z;
//         }
//         publisher_->publish(std::move(msg));
//     }

//     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
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


// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/pose_array.hpp" // 改用 PoseArray
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"
// #include <cmath>
// #include <vector>

// class SensorSim : public rclcpp::Node
// {
// public:
//     SensorSim() : Node("sensor_sim_node")
//     {
//         // 改成發布 PoseArray，跟 ObjectSim 一樣
//         publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/sensors/detected_obstacles", 10);
        
//         tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
//         tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(100), 
//             std::bind(&SensorSim::timer_callback, this));
            
//         RCLCPP_INFO(this->get_logger(), "Sensor Simulation (PoseArray Mode) Started");
//     }

// private:
//     void timer_callback()
//     {
//         // 1. 取得機器人位置
//         double robot_x = 0.0;
//         double robot_y = 0.0;
//         // double robot_yaw = 0.0; // 如果不需要判斷方向，可以先拿掉

//         try {
//             geometry_msgs::msg::TransformStamped t;
//             if (tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero)) {
//                 t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
//             } else {
//                 return;
//             }
//             robot_x = t.transform.translation.x;
//             robot_y = t.transform.translation.y;
            
//             // 如果需要判斷面向，可以解算 quaternion
//             // double qx = t.transform.rotation.x;
//             // double qy = t.transform.rotation.y;
//             // double qz = t.transform.rotation.z;
//             // double qw = t.transform.rotation.w;
//             // robot_yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
            
//         } catch (const tf2::TransformException & ex) {
//             return;
//         }

//         // 2. 定義障礙物位置 (世界座標 map frame)
//         double obs_x = 1.5;
//         double obs_y = 1.0;
        
//         // 3. 計算距離
//         double dist = std::sqrt(std::pow(obs_x - robot_x, 2) + std::pow(obs_y - robot_y, 2));
//         double detection_range = 0.5; // 偵測半徑

//         // 4. 準備訊息
//         geometry_msgs::msg::PoseArray msg;
//         msg.header.stamp = this->now();
//         msg.header.frame_id = "map"; // 直接使用 map frame，跟 ObjectSim 一樣

//         // 5. 邏輯判斷：如果距離夠近，就發布障礙物的位置
//         if (dist < detection_range)
//         {
//             geometry_msgs::msg::Pose p;
//             p.position.x = obs_x;
//             p.position.y = obs_y;
//             p.position.z = 0.0;
//             p.orientation.w = 1.0;
            
//             msg.poses.push_back(p);
            
//             RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
//                 "Detected Obstacle at (%.2f, %.2f) - Distance: %.2f", obs_x, obs_y, dist);
//         }
//         else
//         {
//             // 如果太遠，發布空的 PoseArray，這會讓 Layer 清除障礙物
//             // (msg.poses 保持為空)
//         }

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

class SensorSim : public rclcpp::Node
{
public:
    SensorSim() : Node("sensor_sim_node")
    {
        // 發布 PoseArray (代表感測器打到的障礙物點)
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/sensors/detected_obstacles", 10);
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 10Hz 更新頻率
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&SensorSim::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Sensor Simulation (3-Sensor Raycast) Started");
    }

private:
    void timer_callback()
    {
        // 1. 取得機器人位置與姿態
        double robot_x = 0.0;
        double robot_y = 0.0;
        double robot_yaw = 0.0;

        try {
            geometry_msgs::msg::TransformStamped t;
            if (tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero)) {
                t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            } else {
                return; // TF 還沒準備好
            }
            robot_x = t.transform.translation.x;
            robot_y = t.transform.translation.y;
            
            // 計算 Yaw (方向)
            double qx = t.transform.rotation.x;
            double qy = t.transform.rotation.y;
            double qz = t.transform.rotation.z;
            double qw = t.transform.rotation.w;
            robot_yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
            
        } catch (const tf2::TransformException & ex) {
            return;
        }

        // ==========================================
        // 2. 場景設定 (虛擬障礙物位置)
        // ==========================================
        double obs_x = 1.5;      // 障礙物中心 X
        double obs_y = 0.7;      // 障礙物中心 Y
        double obs_radius = 0.2; // 障礙物半徑 (實體大小)
        
        // 感測器設定
        double max_range = 0.3;  // 感測器最遠能看到多遠
        
        // 定義三個感測器在機器人身上的相對位置 (x, y)
        struct Sensor { std::string name; double x; double y; };
        std::vector<Sensor> sensors = {
            {"Left",   0.075,  0.075}, // 左前方
            {"Center", 0.075,  0.0}, // 正前方
            {"Right",  0.075, -0.075}  // 右前方
        };

        // 準備要發送的 PoseArray
        geometry_msgs::msg::PoseArray msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map"; 

        bool any_hit = false;

        for (const auto& sensor : sensors)
        {
            // A. 計算感測器在世界座標的位置
            double sensor_global_x = robot_x + sensor.x * cos(robot_yaw) - sensor.y * sin(robot_yaw);
            double sensor_global_y = robot_y + sensor.x * sin(robot_yaw) + sensor.y * cos(robot_yaw);

            // B. 射線方向 (假設感測器朝車頭前方)
            double dir_x = cos(robot_yaw);
            double dir_y = sin(robot_yaw);

            // C. 數學運算：射線與圓的交點 (Ray-Circle Intersection)
            // L = CircleCenter - SensorPos
            double Lx = obs_x - sensor_global_x;
            double Ly = obs_y - sensor_global_y;

            // t_ca = L dot Direction
            double t_ca = Lx * dir_x + Ly * dir_y;

            if (t_ca < 0) continue; // 障礙物在背後

            double d2 = (Lx * Lx + Ly * Ly) - (t_ca * t_ca); // 垂直距離平方
            double r2 = obs_radius * obs_radius;

            if (d2 > r2) continue; // 射線沒射中圓柱體

            double t_hc = sqrt(r2 - d2);
            double dist_to_surface = t_ca - t_hc; // 距離表面的距離

            // D. 判斷是否在偵測範圍內
            // if (dist_to_surface > 0 && dist_to_surface < max_range)
            if (dist_to_surface > 0 && dist_to_surface < max_range)
            {
                // 算出「接觸點」的世界座標
                double hit_x = sensor_global_x + dist_to_surface * dir_x;
                double hit_y = sensor_global_y + dist_to_surface * dir_y;

                geometry_msgs::msg::Pose p;
                p.position.x = hit_x;
                p.position.y = hit_y;
                p.position.z = 0.0;
                p.orientation.w = 1.0;
                msg.poses.push_back(p);
                
                any_hit = true;
                
                // Debug Log
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "[%s] Detected at dist: %.2fm", sensor.name.c_str(), dist_to_surface);
            }
        }

        // 只有偵測到才發送，沒偵測到就不發 (Layer 不會清除舊的，除非你有設定清除機制)
        // 為了讓 Layer 能更新(清除)，通常建議發送空陣列代表「現在沒看到東西」
        // 但如果你的邏輯是「只要看到就加上去」，可以一直發。
        // 這裡我們選擇：總是發布 msg (即使是空的)，這樣 Layer 才知道現在狀況
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