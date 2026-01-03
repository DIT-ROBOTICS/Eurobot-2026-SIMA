#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

struct Waypoint {
    double x;
    double y;
    double w;
};

class PatrolNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    PatrolNode() : Node("patrol_node_cpp")
    {
        // 1. Declare and get parameters
        this->declare_parameter<std::string>("waypoints_file", "");
        std::string wp_file_path;
        this->get_parameter("waypoints_file", wp_file_path);

        // If parameter is empty, try to auto-find default path
        if (wp_file_path.empty()) {
            std::string pkg_share = ament_index_cpp::get_package_share_directory("sima_target_client");
            wp_file_path = pkg_share + "/config/waypoints.yaml";
        }

        RCLCPP_INFO(this->get_logger(), "Loading waypoints from: %s", wp_file_path.c_str());

        // 2. Load YAML
        if (!load_waypoints(wp_file_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints file. Shutting down.");
            return; // Initialization failed
        }

        // 3. Initialize Action Client
        client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
            this,
            "navigate_to_pose"
        );

        // 4. Wait for Action Server
        RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 Action Server...");
        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        // 5. Start sending the first point
        send_goal();
    }

private:
    // Helper function to read YAML
    bool load_waypoints(const std::string & filename)
    {
        try {
            YAML::Node config = YAML::LoadFile(filename);
            
            if (!config["waypoints"]) {
                RCLCPP_ERROR(this->get_logger(), "YAML file missing 'waypoints' key");
                return false;
            }

            for (const auto & wp : config["waypoints"]) {
                Waypoint w;
                w.x = wp["x"].as<double>();
                w.y = wp["y"].as<double>();
                w.w = wp["w"].as<double>();
                waypoints_.push_back(w);
            }
            RCLCPP_INFO(this->get_logger(), "Loaded %lu waypoints.", waypoints_.size());
            return true;
        } catch (const YAML::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "YAML Exception: %s", e.what());
            return false;
        }
    }

    void send_goal()
    {
        if (waypoints_.empty()) return;

        auto goal_msg = NavigateToPose::Goal();
        const auto & wp = waypoints_[current_index_];

        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = wp.x;
        goal_msg.pose.pose.position.y = wp.y;
        goal_msg.pose.pose.orientation.w = wp.w;
        goal_msg.pose.pose.orientation.z = 0.0;

        RCLCPP_INFO(this->get_logger(), "Sending Goal [%lu/%lu]: (x=%.2f, y=%.2f)", 
            current_index_ + 1, waypoints_.size(), wp.x, wp.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        // Set up callbacks
        send_goal_options.goal_response_callback = 
            std::bind(&PatrolNode::goal_response_callback, this, std::placeholders::_1);
        
        send_goal_options.result_callback = 
            std::bind(&PatrolNode::result_callback, this, std::placeholders::_1);

        client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandleNav::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            // RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void result_callback(const GoalHandleNav::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                handle_success();
                break;
            case rclcpp_action::ResultCode::ABORTED:
            case rclcpp_action::ResultCode::CANCELED:
                handle_failure();
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                handle_failure();
                break;
        }
    }

    void handle_success()
    {
        RCLCPP_INFO(this->get_logger(), ">>> Success");
        
        // Reset retry count
        retry_count_ = 0;
        // Switch to next point
        current_index_ = (current_index_ + 1) % waypoints_.size();

        // Wait 2 seconds before going to the next point (use timer to avoid blocking main thread)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            [this]() {
                this->timer_->cancel(); // Only execute once
                this->send_goal();
            }
        );
    }

    void handle_failure()
    {
        RCLCPP_WARN(this->get_logger(), ">>> Failed to reach the goal (Failure)");

        if (retry_count_ < max_retries_) {
            retry_count_++;
            RCLCPP_INFO(this->get_logger(), "Attempting retry (%d/%d)...", retry_count_, max_retries_);

            // Slightly delay 1 second before retrying
            timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                [this]() {
                    this->timer_->cancel();
                    this->send_goal(); // Send the same goal again
                }
            );
        } else {
            RCLCPP_ERROR(this->get_logger(), "Retry failed, giving up on this point and moving to the next.");
            retry_count_ = 0;
            current_index_ = (current_index_ + 1) % waypoints_.size();

            // Wait before going to the next point
            timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                [this]() {
                    this->timer_->cancel();
                    this->send_goal();
                }
            );
        }
    }

    // Members
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::vector<Waypoint> waypoints_;
    size_t current_index_ = 0;
    int retry_count_ = 0;
    const int max_retries_ = 1; // Set maximum retries
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}