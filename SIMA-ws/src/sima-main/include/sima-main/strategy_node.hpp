// #ifndef SIMA_MAIN_STRATEGY_NODE_HPP
// #define SIMA_MAIN_STRATEGY_NODE_HPP

// #include <vector>
// #include <string>
// #include <map>
// #include <memory>

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "std_msgs/msg/int16.hpp"
// #include "std_msgs/msg/float32.hpp"
// #include "std_msgs/msg/int32_multi_array.hpp"

// // TODO: Ask about camera's topic and message type

// namespace sima_strategy
// {
// enum class PantryStatus {
//     UNKNOWN = -1,
//     EMPTY = 0,
//     OURS = 1,
//     ENEMY = 2,
//     EVEN = 3,
//     CANATTACK = 4
// };

// struct Pantry {
//     std::string id;
//     PantryStatus status = PantryStatus::UNKNOWN;
//     double sima1_time;
// };

// class SimaStrategyNode : public rclcpp::Node
// {
// public:
//     SimaStrategyNode();    

// private:
//     // Callback function
//     void startCallback(const std_msgs::msg::Int16::SharedPtr msg);
//     void pantryStatusCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

//     void calculateAndAssign();

//     // Publishers for sending goals to 4 Simas
//     std::map<int, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> goal_pubs_;

//     // Subscriber for receiving start signal and pantry status
//     rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr start_sub_;
//     rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr pantry_status_sub_;

//     rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr game_time_sub_;

//     // Helper function
//     void sortByDistance(std::vector<Pantry>& pantries, std::string order = "ascending");
//     void addToSelected(std::vector<Pantry>& pantries, std::vector<Pantry>& selected, PantryStatus target_status);

//     // Variables
//     int sima_id_;
//     int max_pantry_to_target_;

//     int last_start_signal_ = 0;
//     bool has_assigned_ = false;

//     std::vector<int> pantry_status_data_ = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//     // std_msgs::msg::Int32MultiArray pantry_status_data_ = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//     // std_msgs::msg::Int32MultiArray pantry_status_data_;

//     // game time counter
//     rclcpp::TimerBase::SharedPtr watchdog_timer_;
//     rclcpp::Time game_start_time_;
//     rclcpp::Time last_sync_sys_time_;
//     bool game_started_ = false;

//     // mission type
//     int current_mission_type_ = 3;   // 1: Peace, 2: Normal, 3: Aggressive
// };

// } // namespace sima_strategy

// #endif // SIMA_MAIN_STRATEGY_NODE_HPP



#ifndef SIMA_MAIN_STRATEGY_NODE_HPP
#define SIMA_MAIN_STRATEGY_NODE_HPP

#include <vector>
#include <string>
#include <map>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

namespace sima_strategy
{
enum class PantryStatus {
    UNKNOWN = -1,
    EMPTY = 0,
    OURS = 1,
    ENEMY = 2,
    EVEN = 3,
    CANATTACK = 4
};

struct Pantry {
    std::string id;
    PantryStatus status = PantryStatus::UNKNOWN;
    double sima1_time;
};

class SimaStrategyNode : public rclcpp::Node
{
public:
    SimaStrategyNode();    

private:
    void startCallback(const std_msgs::msg::Int16::SharedPtr msg);
    void pantryStatusCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void calculateAndAssign();
    void sortByDistance(std::vector<Pantry>& pantries, std::string order = "ascending");
    void addToSelected(std::vector<Pantry>& pantries, std::vector<Pantry>& selected, PantryStatus target_status);

    std::map<int, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> goal_pubs_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr start_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr pantry_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr game_time_sub_;

    int sima_id_;
    int max_pantry_to_target_;
    size_t dynamic_sima_count_; // 【新增】記錄需要動態分配的 Sima 數量

    // 【新增】固定路線 Sima 參數
    int fixed_sima_id_;
    std::string fixed_sima_pantry_;
    int fixed_sima_mission_;

    int last_start_signal_ = 0;
    bool has_assigned_ = false;

    std::vector<int> pantry_status_data_ = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Time game_start_time_;
    rclcpp::Time last_sync_sys_time_;
    bool game_started_ = false;

    int current_mission_type_ = 2;   // 1: Peace, 2: Normal, 3: Aggressive
};

} // namespace sima_strategy

#endif // SIMA_MAIN_STRATEGY_NODE_HPP