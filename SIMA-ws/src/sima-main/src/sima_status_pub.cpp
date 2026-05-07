#include <chrono>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class SimaStatusPub : public rclcpp::Node
{
public:
  SimaStatusPub()
  : Node("sima_status_pub")
  {
    sima_id_ = this->declare_parameter<int>("sima_id", 1);

    const auto topic_name = makeStatusTopicName(sima_id_);
    status_pub_ = this->create_publisher<std_msgs::msg::Bool>(topic_name, 10);
    timer_ = this->create_wall_timer(1s, std::bind(&SimaStatusPub::publishStatus, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing SIMA status on %s at 1 Hz",
      topic_name.c_str());
  }

private:
  int sima_id_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string makeStatusTopicName(const int sima_id) const
  {
    std::ostringstream topic_stream;
    topic_stream << "/sima_" << std::setw(3) << std::setfill('0') << sima_id << "/status";
    return topic_stream.str();
  }

  void publishStatus()
  {
    std_msgs::msg::Bool msg;
    msg.data = true;
    status_pub_->publish(msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimaStatusPub>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
