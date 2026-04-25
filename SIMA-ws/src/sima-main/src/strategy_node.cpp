// #include "sima-main/strategy_node.hpp" 
// #include <algorithm>

// namespace sima_strategy
// {

// SimaStrategyNode::SimaStrategyNode() : Node("sima_strategy")
// {
//     // 【新增】使用節點的時鐘來初始化，確保時間來源一致 (Source 2)
//     game_start_time_ = this->now();
//     last_sync_sys_time_ = this->now();

//     // Get parameters
//     this->declare_parameter("sima_id", 1); // Default to sima 1 if not set
//     this->declare_parameter("max_pantry_to_target", 4); // Default to targeting
//     this->declare_parameter("mission_type", 2);
//     sima_id_ = this->get_parameter("sima_id").as_int();
//     max_pantry_to_target_ = this->get_parameter("max_pantry_to_target").as_int();
//     current_mission_type_ = this->get_parameter("mission_type").as_int();
//     RCLCPP_INFO(this->get_logger(), "Loaded Mission Type from Launch/Param: %d", current_mission_type_);

//     // 迴圈讀取兩種時間 (為了防呆，啟動時先檢查參數是否存在)
//     std::vector<std::string> pantry_names = {"pantry_A", "pantry_B", "pantry_C", "pantry_D", "pantry_E", "pantry_F", "pantry_G", "pantry_H", "pantry_I", "pantry_J"};
//     for (const auto& name : pantry_names) {
//         this->declare_parameter(name + "_time_normal", 10.0);
//         this->declare_parameter(name + "_time_aggressive", 10.0);
//     }

//     // Create publishers for sending goals to 4 Simas (to be transmitted via Domain Bridge)
//     for (int i = 0; i < max_pantry_to_target_; ++i) {
//         std::string topic_name = "/sima_" + std::to_string(i + sima_id_) + "/goal";
//         goal_pubs_[i] = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
//     }

//     // Create subscriber for game time
//     game_time_sub_ = this->create_subscription<std_msgs::msg::Float32>(
//         "/robot/startup/game_time", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
//             auto current_sys_time = this->now();
            
//             // 將 Float32 的秒數轉為 rclcpp::Duration
//             // msg->data 如果是 10.5 秒，代表比賽已經進行了 10.5 秒
//             rclcpp::Duration elapsed_duration = rclcpp::Duration::from_seconds(msg->data);

//             if (!game_started_) {
//                 game_started_ = true;
//                 RCLCPP_INFO(this->get_logger(), "Game started! Big robot time: %.2f sec", msg->data);
//             }

//             // 【核心邏輯】：動態校正比賽起點 (錨點)
//             game_start_time_ = current_sys_time - elapsed_duration;
//             last_sync_sys_time_ = current_sys_time; // 刷新存活心跳
//     });

//     // Create subscriber for receiving start signal
//     start_sub_ = this->create_subscription<std_msgs::msg::Int16>(
//         "/robot/startup/sima/start", 10, std::bind(&SimaStrategyNode::startCallback, this, std::placeholders::_1));
    
//     // Create subscriber for receiving pantry status
//     pantry_status_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
//         "/pantry_status", 10, std::bind(&SimaStrategyNode::pantryStatusCallback, this, std::placeholders::_1));
    
//     // Create timer
//     watchdog_timer_ = this->create_wall_timer(
//         std::chrono::milliseconds(100), [this]() {
//             if (!game_started_ || has_assigned_) return;
            
//             auto current_sys_time = this->now();
                
//             // 算出從「錨點」到現在經過了幾秒
//             // 不管大機器人有沒有死掉，這個時間都會完美、平滑地繼續往下走
//             double elapsed_sec = (current_sys_time - game_start_time_).seconds();
            
//             // 檢查大機器人是否超過 0.5 秒沒發送訊號 (死亡判定)
//             double time_since_last_sync = (current_sys_time - last_sync_sys_time_).seconds();
//             if (time_since_last_sync > 0.5) {
//                 // 每 2 秒印一次警告，避免洗版。此時系統已經自動切換為內部計時
//                 RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
//                     "Lost time sync with Big Robot for %.2f sec! Using internal clock (Current: %.2f sec).", 
//                     time_since_last_sync, elapsed_sec);
//             }

//             // 88 秒備援機制：無論是靠大機器人同步的時間，還是靠自己繼續數的時間，到了 88 秒就觸發
//             if (elapsed_sec >= 88.0) {
//                 RCLCPP_WARN(this->get_logger(), "88 seconds passed! Triggering auto-deploy!");
//                 calculateAndAssign();
//                 has_assigned_ = true;
//             }
//         });
    
//     RCLCPP_INFO(this->get_logger(), "sima_strategy_node initialized.");
// }

// // version 1

// // void SimaStrategyNode::startCallback(const std_msgs::msg::Int16::SharedPtr msg)
// // {
// //     if (last_start_signal_ == 0 && msg->data > 0) {
// //         last_start_signal_ = msg->data;
// //         RCLCPP_INFO(this->get_logger(), "Received start signal: %d", last_start_signal_);
// //     }
// // }

// // void SimaStrategyNode::pantryStatusCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
// // {
// //     if (last_start_signal_ > 0 && !has_assigned_) {
// //         RCLCPP_INFO(this->get_logger(), "Received pantry status update. Calculating and assigning targets...");
// //         calculateAndAssign(msg);
// //         has_assigned_ = true; // Ensure we only assign once after receiving the start signal
// //     }
// // }

// // version 2 (if not receive pantry status, still have to start when receive start signal)

// void SimaStrategyNode::startCallback(const std_msgs::msg::Int16::SharedPtr msg) {
//     // if (last_start_signal_ == 0 && msg->data > 0) {
//     //     last_start_signal_ = msg->data;
//     //     RCLCPP_INFO(this->get_logger(), "Received start signal: %d", last_start_signal_);
//     //     for (size_t i = 0; i < pantry_status_data_.size(); ++i){
//     //         RCLCPP_INFO(this->get_logger(), "Received pantry data %d = %d", (int)i, pantry_status_data_[i]);
//     //     }
//     //     calculateAndAssign();
//     // }

//     if (!has_assigned_ && msg->data > 0) { // 只要還沒分配過，收到指令就執行
//         // current_mission_type_ = msg->data;

//         // 【修改】先判斷遊戲是否已開始，如果還沒，就當作 0 秒
//         double elapsed = game_started_ ? (this->now() - game_start_time_).seconds() : 0.0;

//         RCLCPP_INFO(this->get_logger(), "Received BIG ROBOT command at %.2f sec! Mission Type: %d", 
//                     elapsed, current_mission_type_);
//         calculateAndAssign();
//         has_assigned_ = true;
//     }
// }

// void SimaStrategyNode::pantryStatusCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg){
//     // if (last_start_signal_ > 0) {
//         for (size_t i = 0; i < msg->data.size(); i++){
//             pantry_status_data_[(int)i] = msg->data[i];
//             // RCLCPP_INFO(this->get_logger(), "Received pantry data %d = %d", (int)i, pantry_status_data_[i]);
//         }
//         // RCLCPP_INFO(this->get_logger(), "Received pantry data finished.");
//     // }
// }

// void SimaStrategyNode::calculateAndAssign()
// {
//     std::vector<Pantry> pantries;
//     std::vector<std::string> pantry_names = {"pantry_A", "pantry_B", "pantry_C", "pantry_D", "pantry_E", "pantry_F", "pantry_G", "pantry_H", "pantry_I", "pantry_J"};

//     for (size_t i = 0; i < pantry_names.size(); ++i) {
//         int status = (i < pantry_status_data_.size()) ? pantry_status_data_[i] : static_cast<int>(PantryStatus::EMPTY);
        
//         // 根據 mission type 決定字尾
//         std::string time_suffix = (current_mission_type_ == 3) ? "_time_aggressive" : "_time_normal";
//         std::string time_param_name = pantry_names[i] + time_suffix;
        
//         double time_val = this->get_parameter(time_param_name).as_double();
        
//         pantries.push_back({pantry_names[i], static_cast<PantryStatus>(status), time_val});
//     }

//     // Create a list to store selected pantries for assignment
//     std::vector<Pantry> selected;
    
//     // sort first by distance (sima1 time) to ensure we pick the nearest ones first
//     sortByDistance(pantries, "ascending");
//     // print out the sorted pantry list for debugging
//     RCLCPP_INFO(this->get_logger(), "Pantries sorted by Sima1 time:");
//     for (const auto& p : pantries) {
//         RCLCPP_INFO(this->get_logger(), "Pantry: %s, Status: %d, Sima1 Time: %.1f", p.id.c_str(), static_cast<int>(p.status), p.sima1_time);
//     }

//     addToSelected(pantries, selected, PantryStatus::CANATTACK); // choose pantry occupied by enemy first
//     addToSelected(pantries, selected, PantryStatus::ENEMY); // choose pantry occupied by enemy first
//     addToSelected(pantries, selected, PantryStatus::EMPTY); // if not enough, choose empty pantry
//     addToSelected(pantries, selected, PantryStatus::EVEN); // if not enough, choose even pantry
//     addToSelected(pantries, selected, PantryStatus::UNKNOWN); // if not enough, choose unknown pantry
//     addToSelected(pantries, selected, PantryStatus::OURS); // if not enough, choose our pantry

//     if (selected.size() < max_pantry_to_target_) {
//         RCLCPP_WARN(this->get_logger(), "Num of valid Pantry is less than 4！Currently: %zu ", selected.size());
//     }

//     sortByDistance(selected, "descending"); // Sort by distance (Sima 1 time) in descending order (farthest to nearest)
//     RCLCPP_INFO(this->get_logger(), "Selected Pantries sorted by Sima1 time:");
//     for (const auto& p : selected) {
//         RCLCPP_INFO(this->get_logger(), "Pantry: %s, Status: %d, Sima1 Time: %.1f", p.id.c_str(), static_cast<int>(p.status), p.sima1_time);
//     }
//     // 4. 分配目標並發送 Topic
//     // index 0 (最遠) -> sima 1
//     // index 1        -> sima 2
//     // index 2        -> sima 3
//     // index 3 (最近) -> sima 4
//     for (size_t i = 0; i < selected.size(); ++i) {
//         int sima_target_id = i + sima_id_;
//         auto msg = std_msgs::msg::String();

//         msg.data = selected[i].id + "|" + std::to_string(current_mission_type_) + std::to_string(static_cast<int>(selected[i].status));  // for example: "pantry_A|1|2"
        
//         goal_pubs_[i]->publish(msg);
//         RCLCPP_INFO(this->get_logger(), "Assign Sima %d go to %s (Mission: %d, Status: %d)", 
//                     sima_target_id, selected[i].id.c_str(), current_mission_type_, static_cast<int>(selected[i].status));
//     }
// }

// void SimaStrategyNode::sortByDistance(std::vector<Pantry>& pantries, std::string order)
// {
//     if (order == "ascending") {
//         std::sort(pantries.begin(), pantries.end(), [&](const Pantry& a, const Pantry& b) {
//                     return a.sima1_time < b.sima1_time;
//                 });
//     } else if (order == "descending") {
//         std::sort(pantries.begin(), pantries.end(), [&](const Pantry& a, const Pantry& b) {
//                     return a.sima1_time > b.sima1_time;
//                 });
//     }
// }

// void SimaStrategyNode::addToSelected(std::vector<Pantry>& pantries, std::vector<Pantry>& selected, PantryStatus target_status)
// {
//     for (const auto& p : pantries) {
//         if (p.status == target_status && selected.size() < max_pantry_to_target_ && p.sima1_time < 15.0) {
//             selected.push_back(p);
//         }
//     }
// }

// } // namespace sima_strategy

// int main(int argc, char ** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<sima_strategy::SimaStrategyNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }


#include "sima-main/strategy_node.hpp" 
#include <algorithm>

namespace sima_strategy
{

SimaStrategyNode::SimaStrategyNode() : Node("sima_strategy")
{
    game_start_time_ = this->now();
    last_sync_sys_time_ = this->now();

    this->declare_parameter("sima_id", 1); 
    this->declare_parameter("max_pantry_to_target", 4); 
    this->declare_parameter("mission_type", 2);

    // 【新增】宣告固定路線 Sima 參數
    this->declare_parameter("fixed_sima_id", 4);
    this->declare_parameter("fixed_sima_pantry", "pantry_G"); // Sima 4 預設去 A 盤
    this->declare_parameter("fixed_sima_mission", 1);         // 預設為保守派 (Peace)

    sima_id_ = this->get_parameter("sima_id").as_int();
    max_pantry_to_target_ = this->get_parameter("max_pantry_to_target").as_int();
    current_mission_type_ = this->get_parameter("mission_type").as_int();
    
    fixed_sima_id_ = this->get_parameter("fixed_sima_id").as_int();
    fixed_sima_pantry_ = this->get_parameter("fixed_sima_pantry").as_string();
    fixed_sima_mission_ = this->get_parameter("fixed_sima_mission").as_int();

    // 【計算動態分配數量】
    dynamic_sima_count_ = 0;
    for (int i = 0; i < max_pantry_to_target_; ++i) {
        if (sima_id_ + i != fixed_sima_id_) {
            dynamic_sima_count_++;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Loaded Mission Type: %d. Dynamic SIMAs: %zu. Fixed SIMA ID: %d -> %s", 
                current_mission_type_, dynamic_sima_count_, fixed_sima_id_, fixed_sima_pantry_.c_str());

    std::vector<std::string> pantry_names = {"pantry_A", "pantry_B", "pantry_C", "pantry_D", "pantry_E", "pantry_F", "pantry_G", "pantry_H", "pantry_I", "pantry_J"};
    for (const auto& name : pantry_names) {
        this->declare_parameter(name + "_time_normal", 10.0);
        this->declare_parameter(name + "_time_aggressive", 10.0);
    }

    for (int i = 0; i < max_pantry_to_target_; ++i) {
        std::string topic_name = "/sima_" + std::to_string(i + sima_id_) + "/goal";
        goal_pubs_[i] = this->create_publisher<std_msgs::msg::String>(topic_name, 10);

        std::string adjust_topic = "/sima_" + std::to_string(i + sima_id_) + "/adjust";
        adjust_pubs_[i] = this->create_publisher<std_msgs::msg::Int16>(adjust_topic, 10);
    }

    game_time_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/robot/startup/game_time", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
            auto current_sys_time = this->now();
            rclcpp::Duration elapsed_duration = rclcpp::Duration::from_seconds(msg->data);

            if (!game_started_) {
                game_started_ = true;
                RCLCPP_INFO(this->get_logger(), "Game started! Big robot time: %.2f sec", msg->data);
            }

            game_start_time_ = current_sys_time - elapsed_duration;
            last_sync_sys_time_ = current_sys_time; 
    });

    start_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        "/robot/startup/sima/start", 10, std::bind(&SimaStrategyNode::startCallback, this, std::placeholders::_1));
    
    pantry_status_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/pantry_status", 10, std::bind(&SimaStrategyNode::pantryStatusCallback, this, std::placeholders::_1));
    
    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), [this]() {
            if (!game_started_ || has_assigned_) return;
            
            auto current_sys_time = this->now();
            double elapsed_sec = (current_sys_time - game_start_time_).seconds();
            double time_since_last_sync = (current_sys_time - last_sync_sys_time_).seconds();
            
            if (time_since_last_sync > 0.5) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                    "Lost time sync with Big Robot for %.2f sec! Using internal clock (Current: %.2f sec).", 
                    time_since_last_sync, elapsed_sec);
            }

            if (elapsed_sec >= 88.0) {
                RCLCPP_WARN(this->get_logger(), "88 seconds passed! Triggering auto-deploy!");
                calculateAndAssign();
                has_assigned_ = true;
            }

            // 【新增邏輯】40秒時觸發排隊動作
            if (elapsed_sec >= 20.0 && !has_triggered_adjust_) {
                has_triggered_adjust_ = true;
                RCLCPP_INFO(this->get_logger(), "40 seconds passed! Starting pre-position sequence.");
                adjust_step_ = 0;
                // 設定每 4 秒換下一台車出發 (可依實際移動時間調整)
                adjust_sequence_timer_ = this->create_wall_timer(
                    std::chrono::seconds(10),
                    std::bind(&SimaStrategyNode::executeAdjustSequence, this)
                );
                executeAdjustSequence(); // 立刻執行第一步 (1號出發)
            }
        });
    
    RCLCPP_INFO(this->get_logger(), "sima_strategy_node initialized.");
}

void SimaStrategyNode::executeAdjustSequence() {
    if (has_assigned_) { 
        // 如果已經收到提早的正式開賽訊號，立刻中斷
        if(adjust_sequence_timer_) adjust_sequence_timer_->cancel();
        return;
    }

    int target_idx = -1;
    // 陣列 index 0,1,2,3 對應機器人 ID 1,2,3,4 (或 11,12,13,14)
    if (adjust_step_ == 0) target_idx = 0;      // 1號 (11號)
    else if (adjust_step_ == 1) target_idx = 1; // 2號 (12號)
    else if (adjust_step_ == 2) target_idx = 3; // 4號 (14號)
    else if (adjust_step_ == 3) target_idx = 2; // 3號 (13號)

    if (target_idx != -1 && adjust_pubs_.count(target_idx)) {
        auto msg = std_msgs::msg::Int16();
        msg.data = 1;
        adjust_pubs_[target_idx]->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Triggering adjust sequence for SIMA ID: %d", sima_id_ + target_idx);
    }

    adjust_step_++;
    if (adjust_step_ >= 4) {
        RCLCPP_INFO(this->get_logger(), "Pre-position sequence completed.");
        adjust_sequence_timer_->cancel();
    }
}

void SimaStrategyNode::startCallback(const std_msgs::msg::Int16::SharedPtr msg) {
    if (!has_assigned_ && msg->data > 0) { 
        double elapsed = game_started_ ? (this->now() - game_start_time_).seconds() : 0.0;

        RCLCPP_INFO(this->get_logger(), "Received BIG ROBOT command at %.2f sec! Mission Type: %d", 
                    elapsed, current_mission_type_);
        calculateAndAssign();
        has_assigned_ = true;
    }
}

void SimaStrategyNode::pantryStatusCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg){
    for (size_t i = 0; i < msg->data.size(); i++){
        pantry_status_data_[(int)i] = msg->data[i];
    }
}

void SimaStrategyNode::calculateAndAssign()
{
    std::vector<Pantry> pantries;
    std::vector<std::string> pantry_names = {"pantry_A", "pantry_B", "pantry_C", "pantry_D", "pantry_E", "pantry_F", "pantry_G", "pantry_H", "pantry_I", "pantry_J"};

    for (size_t i = 0; i < pantry_names.size(); ++i) {
        // 【核心修改】剔除固定部隊的目標盤子，避免衝突
        if (pantry_names[i] == fixed_sima_pantry_) {
            continue;
        }

        int status = (i < pantry_status_data_.size()) ? pantry_status_data_[i] : static_cast<int>(PantryStatus::EMPTY);
        std::string time_suffix = (current_mission_type_ == 3) ? "_time_aggressive" : "_time_normal";
        std::string time_param_name = pantry_names[i] + time_suffix;
        double time_val = this->get_parameter(time_param_name).as_double();
        pantries.push_back({pantry_names[i], static_cast<PantryStatus>(status), time_val});
    }

    std::vector<Pantry> selected;
    sortByDistance(pantries, "ascending");

    addToSelected(pantries, selected, PantryStatus::CANATTACK); 
    addToSelected(pantries, selected, PantryStatus::ENEMY); 
    addToSelected(pantries, selected, PantryStatus::EMPTY); 
    addToSelected(pantries, selected, PantryStatus::EVEN); 
    addToSelected(pantries, selected, PantryStatus::UNKNOWN); 
    addToSelected(pantries, selected, PantryStatus::OURS); 

    if (selected.size() < dynamic_sima_count_) {
        RCLCPP_WARN(this->get_logger(), "Num of valid Pantry is less than %zu！Currently: %zu ", dynamic_sima_count_, selected.size());
    }

    sortByDistance(selected, "descending"); 

    // 【核心修改】發布邏輯：區分固定與動態
    size_t selected_idx = 0;
    for (int i = 0; i < max_pantry_to_target_; ++i) {
        int sima_target_id = i + sima_id_;
        auto msg = std_msgs::msg::String();

        if (sima_target_id == fixed_sima_id_) {
            // 解析固定盤子目前的狀態
            int fixed_status_val = static_cast<int>(PantryStatus::UNKNOWN);
            auto it = std::find(pantry_names.begin(), pantry_names.end(), fixed_sima_pantry_);
            if (it != pantry_names.end()) {
                size_t idx = std::distance(pantry_names.begin(), it);
                fixed_status_val = (idx < pantry_status_data_.size()) ? pantry_status_data_[idx] : static_cast<int>(PantryStatus::EMPTY);
            }

            msg.data = fixed_sima_pantry_ + "|" + std::to_string(fixed_sima_mission_) + "|" + std::to_string(fixed_status_val);
            goal_pubs_[i]->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Assign Sima %d [FIXED] go to %s (Mission: %d, Status: %d)", 
                        sima_target_id, fixed_sima_pantry_.c_str(), fixed_sima_mission_, fixed_status_val);
        } else {
            if (selected_idx < selected.size()) {
                msg.data = selected[selected_idx].id + "|" + 
                           std::to_string(current_mission_type_) + "|" + 
                           std::to_string(static_cast<int>(selected[selected_idx].status));
                goal_pubs_[i]->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Assign Sima %d [DYNAMIC] go to %s (Mission: %d, Status: %d)", 
                            sima_target_id, selected[selected_idx].id.c_str(), current_mission_type_, static_cast<int>(selected[selected_idx].status));
                selected_idx++;
            } else {
                RCLCPP_WARN(this->get_logger(), "Assign Sima %d FAILED (No target left)", sima_target_id);
            }
        }
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
        // 【修正警告】將比較從 max_pantry 改為 dynamic_sima_count_
        if (p.status == target_status && selected.size() < dynamic_sima_count_ && p.sima1_time < 15.0) {
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