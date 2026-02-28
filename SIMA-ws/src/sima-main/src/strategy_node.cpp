#include "sima-main/sima_strategy_node.hpp" 
#include <algorithm>

namespace sima_strategy
{

SimaStrategyNode::SimaStrategyNode() : Node("sima_strategy")
{
    // Get parameters
    this->declare_parameter("sima_id", 1); // Default to sima 1 if not set
    this->declare_parameter("max_pantry_to_target", 4); // Default to targeting
    sima_id_ = this->get_parameter("sima_id").as_int();
    max_pantry_to_target_ = this->get_parameter("max_pantry_to_target").as_int();

    std::vector<std::string> pantry_names = {"pantry_A", "pantry_B", "pantry_C", "pantry_D", "pantry_E", "pantry_F", "pantry_G", "pantry_H", "pantry_I", "pantry_J"};
    for (const auto& name : pantry_names) {
        this->declare_parameter(name + "_time", 10.0);
    }

    // Create publishers for sending goals to 4 Simas (to be transmitted via Domain Bridge)
    for (int i = 0; i < max_pantry_to_target_; ++i) {
        std::string topic_name = "/sima_" + std::to_string(i + sima_id_) + "/goal";
        goal_pubs_[i] = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
    }

    // Create subscriber for receiving start signal
    start_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        "/robot/startup/sima/start", 10, std::bind(&SimaStrategyNode::startCallback, this, std::placeholders::_1));
    
    // Create subscriber for receiving pantry status
    pantry_status_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "/pantry_status", 10, std::bind(&SimaStrategyNode::pantryStatusCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "sima_strategy_node initialized.");
}

void SimaStrategyNode::startCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
    if (last_start_signal_ == 0 && msg->data > 0) {
        last_start_signal_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received start signal: %d", last_start_signal_);
    }
}

void SimaStrategyNode::pantryStatusCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
    if (last_start_signal_ > 0 && !has_assigned_) {
        RCLCPP_INFO(this->get_logger(), "Received pantry status update. Calculating and assigning targets...");
        calculateAndAssign(msg);
        has_assigned_ = true; // Ensure we only assign once after receiving the start signal
    }
}

void SimaStrategyNode::calculateAndAssign(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
    std::vector<Pantry> pantries;
    std::vector<std::string> pantry_names = {"pantry_A", "pantry_B", "pantry_C", "pantry_D", "pantry_E", "pantry_F", "pantry_G", "pantry_H", "pantry_I", "pantry_J"};

    for (size_t i = 0; i < pantry_names.size(); ++i) {
        int status = (i < msg->data.size()) ? msg->data[i] : static_cast<int>(PantryStatus::EMPTY);
        
        std::string time_param_name = pantry_names[i] + "_time";
        double time_val = this->get_parameter(time_param_name).as_double();
        
        pantries.push_back({pantry_names[i], static_cast<PantryStatus>(status), time_val});
        
        RCLCPP_DEBUG(this->get_logger(), "load %s, status: %d, Sima1 time: %.1f", pantry_names[i].c_str(), status, time_val);
    }

    // Create a list to store selected pantries for assignment
    std::vector<Pantry> selected;
    
    // sort first by distance (sima1 time) to ensure we pick the nearest ones first
    sortByDistance(pantries, "ascending");

    addToSelected(pantries, selected, PantryStatus::ENEMY); // choose pantry occupied by enemy first
    addToSelected(pantries, selected, PantryStatus::EMPTY); // if not enough, choose empty pantry
    addToSelected(pantries, selected, PantryStatus::EVEN); // if not enough, choose even pantry
    addToSelected(pantries, selected, PantryStatus::UNKNOWN); // if not enough, choose unknown pantry
    addToSelected(pantries, selected, PantryStatus::OURS); // if not enough, choose our pantry

    if (selected.size() < 4) {
        RCLCPP_WARN(this->get_logger(), "Num of valid Pantry is less than 4！Currently: %zu ", selected.size());
    }

    sortByDistance(selected, "descending"); // Sort by distance (Sima 1 time) in descending order (farthest to nearest)

    // 4. 分配目標並發送 Topic
    // index 0 (最遠) -> sima 1
    // index 1        -> sima 2
    // index 2        -> sima 3
    // index 3 (最近) -> sima 4
    for (size_t i = 0; i < selected.size(); ++i) {

        int sima_target_id = i + sima_id_;
        auto msg = std_msgs::msg::String();
        msg.data = selected[i].id; // 傳送如 "pantry_A" 的字串
        
        goal_pubs_[i]->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Assign Sima %d go to %s.", sima_target_id, selected[i].id.c_str());
    }
}

void SimaStrategyNode::sortByDistance(std::vector<Pantry>& pantries, std::string order)
{
    if (order == "ascending") {
        std::sort(pantries.begin(), pantries.end(), [&](const Pantry& a, const Pantry& b) {
                    return a.sima1_time < b.sima1_time;
                });
    } else if (order == "descending") {
        std::sort(pantries.begin(), pantries.end(), [&](const Pantry& a, const Pantry& b) {
                    return a.sima1_time > b.sima1_time;
                });
    }
}

void SimaStrategyNode::addToSelected(std::vector<Pantry>& pantries, std::vector<Pantry>& selected, PantryStatus target_status)
{
    for (const auto& p : pantries) {
        if (p.status == target_status && selected.size() < 4) {
            selected.push_back(p);
        }
    }
}

} // namespace sima_strategy

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sima_strategy::SimaStrategyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}