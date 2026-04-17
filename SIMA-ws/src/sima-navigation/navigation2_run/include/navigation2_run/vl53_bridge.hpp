#ifndef VL53_BRIDGE_HPP_
#define VL53_BRIDGE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <string>
#include <map>

class VL53Bridge : public rclcpp::Node
{
public:
    VL53Bridge();

private:
    struct SensorConfig {
        std::string name;
        double x_offset;   
        double y_offset;   
        double yaw_angle;  
    };

    // sensor memory structure
    struct SensorMemory {
        float last_valid_dist;
        rclcpp::Time last_seen_time;
        bool is_valid;
    };

    void rawDataCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void loadParameters();

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_raw_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;

    std::vector<SensorConfig> sensors_;
    std::map<int, SensorMemory> memory_; // store each sensor's data(memory)

    // parameters
    double min_trust_dist_ = 0.05;  // sensors data can trust range lower bound
    double max_trust_dist_ = 0.50;  // sensors data can trust range upper bound (ignore data more than this range)
    double memory_duration_ = 0.5;  // memory duration time(s)

    double raytrace_max_range_ = 2.5;
    double half_fov_mark_deg_ = 3.0;
    double half_fov_clear_deg_ = 13.5;
    double gap_fill_tolerance_ = 0.1;
    int smear_rays_ = 3;
};

#endif // VL53_BRIDGE_HPP_