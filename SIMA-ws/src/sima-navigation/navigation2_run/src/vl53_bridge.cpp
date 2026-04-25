#include "navigation2_run/vl53_bridge.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

constexpr double deg2rad(double deg) { return deg * M_PI / 180.0; }

VL53Bridge::VL53Bridge() : Node("vl53_bridge_node")
{
    loadParameters();

    rclcpp::QoS qos_sensor(10);
    qos_sensor.best_effort();
    
    sub_raw_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/sensors/raw_ranges", qos_sensor, 
        std::bind(&VL53Bridge::rawDataCallback, this, std::placeholders::_1));

    pub_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/sensors/vl53_scan", 10);

    // initialize memory
    for (size_t i = 0; i < sensors_.size(); ++i) {
        memory_[i] = {0.0f, this->now(), false};
    }

    RCLCPP_INFO(this->get_logger(), "VL53 Bridge (Virtual LiDAR + Memory) Started.");
}

void VL53Bridge::loadParameters()
{
    sensors_ = {
        {"Left",   0.04867,  0.03152, deg2rad(33.57)},
        {"Center", 0.053,  0.0,       deg2rad(0.0)},
        {"Right",  0.04867, -0.03152, deg2rad(-33.57)}
    };

    // The trusted distance range
    this->declare_parameter("min_trust_dist", 0.05);
    this->get_parameter("min_trust_dist", min_trust_dist_);
    this->declare_parameter("max_trust_dist", 0.65);
    this->get_parameter("max_trust_dist", max_trust_dist_);
    
    // memory duration
    this->declare_parameter("memory_duration", 0.05);
    this->get_parameter("memory_duration", memory_duration_);

    // Other params
    this->declare_parameter("raytrace_max_range", 2.5);
    this->get_parameter("raytrace_max_range", raytrace_max_range_);

    this->declare_parameter("half_fov_mark_deg", 3.0);
    this->get_parameter("half_fov_mark_deg", half_fov_mark_deg_);

    this->declare_parameter("half_fov_clear_deg", 13.5);
    this->get_parameter("half_fov_clear_deg", half_fov_clear_deg_);

    this->declare_parameter("gap_fill_tolerance", 0.1);
    this->get_parameter("gap_fill_tolerance", gap_fill_tolerance_);

    this->declare_parameter("smear_rays", 3);
    this->get_parameter("smear_rays", smear_rays_);
}

void VL53Bridge::rawDataCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() != sensors_.size()) return;

    // --- 加入以下這兩行來對調左右數據 ---
    std::vector<float> corrected_data = msg->data;
    std::swap(corrected_data[0], corrected_data[2]); 
    // ------------------------------------

    rclcpp::Time current_time = this->now();
    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header.stamp = current_time;
    scan_msg.header.frame_id = "base_link"; 

    // Define Lidar range: slightly wider than sensors width limit (-45 degree ~ +45 degree)
    scan_msg.angle_min = deg2rad(-45.0);
    scan_msg.angle_max = deg2rad(45.0);
    scan_msg.angle_increment = deg2rad(0.5);
    scan_msg.time_increment = 0.0;
    scan_msg.scan_time = 0.1;
    scan_msg.range_min = min_trust_dist_;
    scan_msg.range_max = raytrace_max_range_; // use to shoot long distance eliminate laser

    int num_rays = std::round((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1;
    
    // Fill the lists with NAN -> So that the old space in blind spot won't be updated
    // Nav2 遇到 NaN 會完全忽略（不畫牆、也不拆牆），這保護了盲區內的舊牆壁不被誤刪。
    scan_msg.ranges.assign(num_rays, std::numeric_limits<float>::quiet_NaN());

    double half_fov_mark = deg2rad(half_fov_mark_deg_);       // draw wall marker
    double half_fov_clear = deg2rad(half_fov_clear_deg_);     // erase wall marker

    for (size_t i = 0; i < sensors_.size(); ++i) {
        // float raw_dist = msg->data[i];
        // --- 將 msg->data[i] 改為讀取對調後的 corrected_data[i] ---
        float raw_dist = corrected_data[i]; 
        // -----------------------------------------------------------

        float output_dist = std::numeric_limits<float>::infinity(); // initial output(inf)

        if (raw_dist > 0.01f && raw_dist <= max_trust_dist_) {
            float clamped_dist = std::max(raw_dist, (float)min_trust_dist_);
            
            // if dist smaller than min trust dist: still have to draw at min trust dist; otherwise it will not see the obstacle
            memory_[i].last_valid_dist = clamped_dist;
            memory_[i].last_seen_time = current_time;
            memory_[i].is_valid = true;
            output_dist = clamped_dist;

        } else {
            double age = (current_time - memory_[i].last_seen_time).seconds();
            if (memory_[i].is_valid && age < memory_duration_) {
                output_dist = memory_[i].last_valid_dist;
            } else {
                memory_[i].is_valid = false;
                output_dist = std::numeric_limits<float>::infinity();
            }
        }

        // Optimization: Dynamically determine the angle range that this sensor should affect
        // If it's inf (to clear), use a large brush; if it's numerical (to draw a wall), use a fine brush.
        double active_half_fov = std::isinf(output_dist) ? half_fov_clear : half_fov_mark;

        // 2. Fill the calculated distance into the corresponding narrow sector.
        const auto& sensor = sensors_[i];
        double start_angle = sensor.yaw_angle - active_half_fov;
        double end_angle = sensor.yaw_angle + active_half_fov;

        int start_idx = std::round((start_angle - scan_msg.angle_min) / scan_msg.angle_increment);
        int end_idx = std::round((end_angle - scan_msg.angle_min) / scan_msg.angle_increment);

        start_idx = std::max(0, start_idx);
        end_idx = std::min(num_rays - 1, end_idx);

        for (int j = start_idx; j <= end_idx; ++j) {
            if (std::isinf(output_dist)) {
                 scan_msg.ranges[j] = std::numeric_limits<float>::infinity();
            } else {
                 scan_msg.ranges[j] = output_dist + sensor.x_offset;
            }
        }
    }

    auto check_and_fill_gap = [&](int idx_a, int idx_b) {
        if (memory_[idx_a].is_valid && memory_[idx_b].is_valid) {
            float dist_a = memory_[idx_a].last_valid_dist;
            float dist_b = memory_[idx_b].last_valid_dist;
            
            if (std::abs(dist_a - dist_b) < gap_fill_tolerance_) {
                const auto& s_a = sensors_[idx_a];
                const auto& s_b = sensors_[idx_b];

                int ray_a = std::round((s_a.yaw_angle - scan_msg.angle_min) / scan_msg.angle_increment);
                int ray_b = std::round((s_b.yaw_angle - scan_msg.angle_min) / scan_msg.angle_increment);

                int start_fill = std::min(ray_a, ray_b);
                int end_fill = std::max(ray_a, ray_b);
                
                start_fill = std::max(0, start_fill);
                end_fill = std::min(num_rays - 1, end_fill);

                for (int k = start_fill; k <= end_fill; ++k) {
                    float ratio = (float)(k - start_fill) / (end_fill - start_fill);
                    float dist_start = (start_fill == ray_a) ? dist_a : dist_b;
                    float dist_end = (end_fill == ray_a) ? dist_a : dist_b;
                    
                    float interp_dist = dist_start + ratio * (dist_end - dist_start);
                    float avg_offset = (s_a.x_offset + s_b.x_offset) / 2.0f;
                    
                    // 【Thickening Magic】: We not only draw interp_dist, but if possible,
                    // In LaserScan, we take the "nearest distance".
                    // To thicken the wall, we can also fill in the "adjacent angles" around this angle with the same distance.
                    // This way, although the depth doesn't increase, the lateral density will be extremely high, doubling the tear resistance.
                    float final_dist = interp_dist + avg_offset;

                    // Optimization 3: Heavy Smearing
                    // Instead of drawing a single point, extend 3 rays to the left and right (covering a total of 7 rays, approximately 3.5 degrees)
                    // This way, even if coordinate transformation causes floating-point errors, this wall will absolutely not be erased!
                   
                    for (int m = -smear_rays_; m <= smear_rays_; ++m) {
                        int target_idx = k + m;
                        if (target_idx >= 0 && target_idx < num_rays) {
                            if (std::isinf(scan_msg.ranges[target_idx]) || 
                                std::isnan(scan_msg.ranges[target_idx]) || 
                                final_dist < scan_msg.ranges[target_idx]) 
                            {
                                scan_msg.ranges[target_idx] = final_dist;
                            }
                        }
                    }
                }
            }
        }
    };

    // Check if there is a wall between the left (0) and the middle (1)
    check_and_fill_gap(0, 1);
    // Check if there is a wall between the middle (1) and the right (2)
    check_and_fill_gap(1, 2);

    pub_scan_->publish(scan_msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VL53Bridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}