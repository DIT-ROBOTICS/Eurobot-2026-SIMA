#include "navigation2_run/vl53_bridge.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// transform degree to radian
constexpr double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

VL53Bridge::VL53Bridge() : Node("vl53_bridge_node")
{
    // 1. Initialize TF buffer
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 2. load parameters
    loadParameters();

    // 3. Establish communication
    // QoS set to Best Effort because losing one or two frames of sensor data is acceptable; timeliness is more important
    rclcpp::QoS qos_sensor(10);
    qos_sensor.best_effort();
    
    sub_raw_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/sensors/raw_ranges", qos_sensor, 
        std::bind(&VL53Bridge::rawDataCallback, this, std::placeholders::_1));

    pub_obstacles_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/sensors/detected_obstacles", 10);

    RCLCPP_INFO(this->get_logger(), "VL53 Bridge Node Started. Listening for raw ranges...");
}

void VL53Bridge::loadParameters()
{
    // default sensor configurations
    // [0:Left, 1:Center, 2:Right]
    sensors_ = {
        {"Left",   0.04867,  0.03152, deg2rad(33.57)},
        {"Center", 0.053,  0.0,   deg2rad(0.0)},
        {"Right",  0.04867, -0.03152, deg2rad(-33.57)}
    };

    // TODO: read param file to override default sensor configs
    this->declare_parameter("trigger_distance", 0.5);
    this->get_parameter("trigger_distance", trigger_distance_);
}

void VL53Bridge::rawDataCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    // 1. check data size
    if (msg->data.size() != sensors_.size()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
            "Received data size (%ld) does not match sensor count (%ld)!", 
            msg->data.size(), sensors_.size());
        return;
    }

    // 2. ready output message
    geometry_msgs::msg::PoseArray output_msg;
    output_msg.header.stamp = this->now();
    output_msg.header.frame_id = "map"; // transform to map frame

    // 3. check if TF is available
    // We need to transform points from base_link to map
    // Here we don't query Transform, but directly use tf_buffer->transform() function
    if (!tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
            "Waiting for TF (map -> base_link)... Cannot publish obstacles.");
        return;
    }

    // 4. iterate each sensor
    for (size_t i = 0; i < sensors_.size(); ++i) {
        if (i == 0) continue;   // disable left sensor
        float dist = msg->data[i];

        // check if it is a valid obstacle
        if (dist > min_valid_dist_ && dist < trigger_distance_) {
            
            const auto& sensor = sensors_[i];

            // A. calculate obstacle position in robot frame (base_link)
            // Obs_X = installation X + distance * cos(installation angle)
            // Obs_Y = installation Y + distance * sin(installation angle)
            double local_x = sensor.x_offset + dist * std::cos(sensor.yaw_angle);
            double local_y = sensor.y_offset + dist * std::sin(sensor.yaw_angle);

            // B. create PointStamped (to let TF know this point is in base_link)
            geometry_msgs::msg::PointStamped point_in_base, point_in_map;
            point_in_base.header.frame_id = "base_link";
            point_in_base.header.stamp = rclcpp::Time(0); // latest time
            point_in_base.point.x = local_x;
            point_in_base.point.y = local_y;
            point_in_base.point.z = 0.0;

            try {
                // C. perform coordinate transformation (base_link -> map)
                // This line automatically handles the robot's position and orientation
                point_in_map = tf_buffer_->transform(point_in_base, "map");

                // D. add to output list
                geometry_msgs::msg::Pose pose_map;
                pose_map.position = point_in_map.point;
                pose_map.orientation.w = 1.0; // obstacle is a point, orientation is not important
                
                output_msg.poses.push_back(pose_map);

                // Debug
                // RCLCPP_INFO(this->get_logger(), "[%s] Hit at %.2fm -> Map(%.2f, %.2f)", 
                //     sensor.name.c_str(), dist, point_in_map.point.x, point_in_map.point.y);

            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "TF Transform failed: %s", ex.what());
                continue;
            }
        }
    }

    // 5. publish
    // Even if there are no obstacles, consider whether to publish an empty message (depending on your SensorLayer logic)
    // Here we choose: always publish, so the monitoring side knows the node is alive
    pub_obstacles_->publish(output_msg);






    // // 1. 鎖定感測器數據的時間戳 (這是關鍵！)
    // // 因為 Float32MultiArray 沒有 header，我們假設收到當下就是觀測時間
    // rclcpp::Time sensor_time = this->now();

    // geometry_msgs::msg::PoseArray output_msg;
    // output_msg.header.stamp = sensor_time; // 輸出訊息跟隨感測器時間
    // output_msg.header.frame_id = "map";

    // // 2. 等待 TF (LookupTransform with Timeout)
    // // 我們不能用 transform() 直接轉，因為它預設不支援 timeout
    // // 我們需要等待定位系統 (EKF/Camera) 發布這個時間點的 TF，這通常需要幾毫秒到幾百毫秒
    // geometry_msgs::msg::TransformStamped transform_stamped;
    // try {
    //     // 設定等待時間 (例如 0.2秒)，這取決於你的定位延遲有多大
    //     // 如果定位系統延遲超過這個時間，這幀數據就會被捨棄，避免畫錯
    //     transform_stamped = tf_buffer_->lookupTransform(
    //         "map", "base_link",
    //         sensor_time,
    //         rclcpp::Duration::from_seconds(0.2)); 
    // } catch (const tf2::TransformException & ex) {
    //     // 如果等不到 TF (定位太慢或斷了)，就跳過這一次
    //     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
    //         "Wait for TF failed (latency too high?): %s", ex.what());
    //     return;
    // }

    // // 3. 遍歷每個感測器並轉換
    // for (size_t i = 0; i < sensors_.size(); ++i) {
    //     if (i == 0) continue; // disable left sensor
    //     float dist = msg->data[i];

    //     if (dist > min_valid_dist_ && dist < trigger_distance_) {
    //         const auto& sensor = sensors_[i];

    //         // A. 計算 base_link 上的局部座標
    //         double local_x = sensor.x_offset + dist * std::cos(sensor.yaw_angle);
    //         double local_y = sensor.y_offset + dist * std::sin(sensor.yaw_angle);

    //         // B. 準備轉換的點
    //         // 注意：這裡不需要再填 stamp，因為我們會用上面拿到的 transform_stamped 直接算
    //         geometry_msgs::msg::PointStamped point_in_base;
    //         point_in_base.point.x = local_x;
    //         point_in_base.point.y = local_y;
    //         point_in_base.point.z = 0.0;

    //         // C. 執行座標轉換 (使用 tf2::doTransform)
    //         geometry_msgs::msg::PointStamped point_in_map;
    //         tf2::doTransform(point_in_base, point_in_map, transform_stamped);

    //         // D. 加入輸出列表
    //         geometry_msgs::msg::Pose pose_map;
    //         pose_map.position = point_in_map.point;
    //         pose_map.orientation.w = 1.0;
    //         output_msg.poses.push_back(pose_map);
    //     }
    // }

    // // 4. 發布
    // pub_obstacles_->publish(output_msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VL53Bridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}