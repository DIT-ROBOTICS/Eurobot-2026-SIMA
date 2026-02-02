// This node republishes TF messages, removing a specified prefix from frame IDs.
// Useful in multi-robot setups where TF frames are namespaced.
// Details:
// - Subscribes to /tf_bridged and /tf_static_bridged topics send by the domain bridge from domain 1 (Gazebo simulator).
// - Republishes to /tf and /tf_static after stripping the configured prefix from frame IDs
// - For example, if the prefix is "sima1/", a frame "sima1/base_link" becomes "base_link".
// - Only need to run this node in simulation. In real robot, frames are already correct since /tf and /tf_static are published in each robots' own domain.

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using std::placeholders::_1;

class TFRepublisher : public rclcpp::Node
{
public:
  TFRepublisher()
  : Node("tf_republisher")
  {
    // Declare and get parameters
    this->declare_parameter<std::string>("remove_prefix", "sima1/");
    this->get_parameter("remove_prefix", remove_prefix_);

    RCLCPP_INFO(this->get_logger(), "TF Republisher Started (C++). Stripping prefix: '%s'", remove_prefix_.c_str());

    // Setting QoS for tf_static (needs Transient Local)
    rclcpp::QoS static_qos(50);
    static_qos.transient_local();
    static_qos.reliable();

    // Create publishers for /tf and /tf_static
    pub_tf_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 100);
    pub_tf_static_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", static_qos);

    // Create subscriptions for /tf_bridged and /tf_static_bridged
    sub_tf_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_bridged", 100, std::bind(&TFRepublisher::tf_callback, this, _1));

    sub_tf_static_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static_bridged", static_qos, std::bind(&TFRepublisher::tf_static_callback, this, _1));
  }

private:
  bool has_prefix(const std::string & frame_id)
  {
    return frame_id.rfind(remove_prefix_, 0) == 0;
  }

  // Remove prefix string
  std::string clean_frame(const std::string & frame_id)
  {
    // Check if the string starts with remove_prefix_
    if (frame_id.rfind(remove_prefix_, 0) == 0) {
      return frame_id.substr(remove_prefix_.length());
    }
    return frame_id;
  }

  // Process message and repack
  tf2_msgs::msg::TFMessage process_msg(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    tf2_msgs::msg::TFMessage new_msg;
    
    // Pre-allocate space to improve performance
    new_msg.transforms.reserve(msg->transforms.size());

    for (const auto & t : msg->transforms) {
        if (has_prefix(t.header.frame_id) || has_prefix(t.child_frame_id)) {
            auto new_t = t;
            new_t.header.frame_id = clean_frame(t.header.frame_id);
            new_t.child_frame_id = clean_frame(t.child_frame_id);
            new_msg.transforms.push_back(new_t);
        }
    }

    return new_msg;
  }

  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    auto new_msg = process_msg(msg);
    if (!new_msg.transforms.empty()) {
        pub_tf_->publish(new_msg);
    }
  }

  void tf_static_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    auto new_msg = process_msg(msg);
    pub_tf_static_->publish(new_msg);
  }

  std::string remove_prefix_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_static_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_static_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFRepublisher>());
  rclcpp::shutdown();
  return 0;
}