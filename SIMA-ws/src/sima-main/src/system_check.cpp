#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "btcpp_ros2_interfaces/srv/start_up_srv.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

class SystemCheck : public rclcpp::Node
{
public:
  SystemCheck()
  : Node("SystemCheck"), is_main_ready_(false)
  {
    ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/robot/startup/are_you_ready", 10,
      std::bind(&SystemCheck::readyCallback, this, std::placeholders::_1));

    ready_srv_client_ = this->create_client<btcpp_ros2_interfaces::srv::StartUpSrv>(
      "/robot/startup/ready_signal");

    RCLCPP_INFO(this->get_logger(), "\033[1;35m SystemCheck started, waiting for startup plan... \033[0m");
  }

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp::Client<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedPtr ready_srv_client_;
  bool is_main_ready_;

  void readyCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg == nullptr || is_main_ready_)
      return;

    // RCLCPP_INFO(this->get_logger(), "\033[1;35m Received startup plan message, processing... \033[0m");

    // Retry until service is ready
    while (!ready_srv_client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Waiting for ReadySignal service...");
      if (!rclcpp::ok()) return;
    }

    // system check logic can be added here

    // All systems ready
    is_main_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "\033[1;32m All systems ready !!! \033[0m");
    sendReadySignal();
  }

  void sendReadySignal(void)
  {
    auto request = std::make_shared<btcpp_ros2_interfaces::srv::StartUpSrv::Request>();
    request->group = 5;  // group 5 is for little sima
    request->state = static_cast<int>(1);  // state 1 is for READY

    // RCLCPP_INFO(this->get_logger(), "\033[1;35m Sending ReadySignal (group=%d, state=%d)... \033[0m", group, state);

    int captured_group = 5;
    ready_srv_client_->async_send_request(request,
      [this, captured_group](rclcpp::Client<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedFuture future) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "\033[1;32m ReadySignal SUCCESS: group=%d \033[0m", response->group);
            is_main_ready_ = false; // Reset for next system check process
        } else {
            RCLCPP_WARN(this->get_logger(), "ReadySignal FAILED");
        }
      });
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SystemCheck>();
  rclcpp::spin(node);
  return 0;
}
