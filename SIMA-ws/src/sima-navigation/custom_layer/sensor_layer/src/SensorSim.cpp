#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <cmath>
#include <vector>
#include <string>

// Convert angle from degrees to radians
inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

class SensorSim : public rclcpp::Node
{
public:
    SensorSim() : Node("sensor_sim_node")
    {
        // Publish PoseArray (representing obstacle points detected by sensors)
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/sensors/detected_obstacles", 10);
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 10Hz update frequency (simulating sensor refresh rate)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&SensorSim::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Sensor Simulation (Angled VL53) Started");
    }

private:
    void timer_callback()
    {
        // ==========================================
        // 1. Get robot position and pose (Robot State)
        // ==========================================
        double robot_x = 0.0;
        double robot_y = 0.0;
        double robot_yaw = 0.0;

        try {
            geometry_msgs::msg::TransformStamped t;
            if (tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero)) {
                t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            } else {
                return; // TF not ready yet, skip this loop
            }
            robot_x = t.transform.translation.x;
            robot_y = t.transform.translation.y;
            
            // Calculate Yaw (robot orientation)
            double qx = t.transform.rotation.x;
            double qy = t.transform.rotation.y;
            double qz = t.transform.rotation.z;
            double qw = t.transform.rotation.w;
            robot_yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
            
        } catch (const tf2::TransformException & ex) {
            return;
        }

        // ==========================================
        // 2. Simulate environment setup (Virtual Obstacle)
        // ==========================================
        // Place a circular obstacle on the map
        double obs_x = 1.5;      // Obstacle center X
        double obs_y = 0.5;      // Obstacle center Y (slightly to the left, so left sensor can detect it easily)
        double obs_radius = 0.3; // Obstacle radius (like a large pillar)
        
        // Sensor parameters (VL53L1X ~2-4m, VL53L0X ~1-2m, set to 1.5m here)
        double max_range = 0.5; 

        // ==========================================
        // 3. Sensor configuration (Key modification)
        // ==========================================
        struct Sensor { 
            std::string name; 
            double x;    // Installation position X (relative to base_link)
            double y;    // Installation position Y (relative to base_link)
            double yaw;  // Installation angle (relative to base_link)
        };

        std::vector<Sensor> sensors = {
            // Left sensor: installed at left-front, pointing 45 degrees to the left
            {"Left",   0.075,  0.075, deg2rad(45.0)}, 
            
            // Center sensor: installed at front center, pointing 0 degrees
            {"Center", 0.075,  0.0,   deg2rad(0.0)}, 
            
            // Right sensor: installed at right-front, pointing -45 degrees to the right
            {"Right",  0.075, -0.075, deg2rad(-45.0)}
        };

        // Prepare PoseArray message
        geometry_msgs::msg::PoseArray msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map"; // We calculated world coordinates, send directly in map frame

        bool any_hit = false;

        for (const auto& sensor : sensors)
        {
            // A. Calculate sensor "body" position in world coordinates
            // Rigid body transformation matrix calculation
            double sensor_global_x = robot_x + sensor.x * cos(robot_yaw) - sensor.y * sin(robot_yaw);
            double sensor_global_y = robot_y + sensor.x * sin(robot_yaw) + sensor.y * cos(robot_yaw);

            // B. Calculate sensor "ray direction" (Key modification!)
            // Ray angle = robot angle + sensor installation angle
            double global_sensor_angle = robot_yaw + sensor.yaw;
            double dir_x = cos(global_sensor_angle);
            double dir_y = sin(global_sensor_angle);

            // 
            // C. Mathematical calculation: Ray-Circle Intersection
            // L = CircleCenter - SensorPos (vector from sensor to circle center)
            double Lx = obs_x - sensor_global_x;
            double Ly = obs_y - sensor_global_y;

            // t_ca = L dot Direction (distance of circle center projection on ray)
            double t_ca = Lx * dir_x + Ly * dir_y;

            if (t_ca < 0) continue; // Obstacle is behind the sensor

            double d2 = (Lx * Lx + Ly * Ly) - (t_ca * t_ca); // Squared shortest distance from circle center to ray
            double r2 = obs_radius * obs_radius;

            if (d2 > r2) continue; // Ray completely misses the circle

            double t_hc = sqrt(r2 - d2); // Distance from tangent point to circle center
            double dist_to_surface = t_ca - t_hc; // Final distance: sensor to circle surface

            // D. Check if within valid detection range
            if (dist_to_surface > 0 && dist_to_surface < max_range)
            {
                // Calculate "hit point" world coordinates
                double hit_x = sensor_global_x + dist_to_surface * dir_x;
                double hit_y = sensor_global_y + dist_to_surface * dir_y;

                geometry_msgs::msg::Pose p;
                p.position.x = hit_x;
                p.position.y = hit_y;
                p.position.z = 0.0; // On the ground
                p.orientation.w = 1.0;
                msg.poses.push_back(p);
                
                any_hit = true;
                
                // Debug Log (use Throttle to prevent spam)
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "HIT! [%s] Dist: %.2fm at (%.2f, %.2f)", 
                    sensor.name.c_str(), dist_to_surface, hit_x, hit_y);
            }
        }

        // Publish results (publish even if empty, so SensorLayer knows there's nothing)
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