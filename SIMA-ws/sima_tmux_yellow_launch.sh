#!/bin/bash

# 確保你執行此腳本前已經 source 過 ROS 2 環境
# source /opt/ros/humble/setup.bash
# source install/setup.bash

SESSION_NAME="sima_run"
MISSION_TYPE="${1:-${MISSION_TYPE:-2}}"

# 如果已經有同名的 tmux session，先關閉確保環境乾淨
tmux kill-session -t $SESSION_NAME 2>/dev/null

# 建立新的 tmux session (先背景執行)
tmux new-session -d -s $SESSION_NAME

# 將 tmux 畫面切割為 3 個區塊
# 預設開啟的第一個視窗為 Pane 0 (左半部)
tmux split-window -h -t $SESSION_NAME:0
# 將右半部向下切一半，產生 Pane 1 (右上) 與 Pane 2 (右下)
tmux split-window -v -t $SESSION_NAME:0.1

# 啟用滑鼠支援 (可選，方便你拖曳調整各個區塊的大小)
tmux set-option -t $SESSION_NAME mouse on

# 印出環境變數與參數檔資訊
echo -e "\033[93mTeam: yellow, ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}, using params file: main_params_sima_yellow.yaml\033[0m"
echo -e "\033[93mMission type: ${MISSION_TYPE} (1: Peace, 2: Normal, 3: Aggressive)\033[0m"
sleep 1

# 定義參數檔的路徑 (讓 tmux 內的終端機動態解析)
PARAMS_FILE="\$(ros2 pkg prefix sima-main)/share/sima-main/params/main_params_sima_yellow.yaml"

# ================= 區塊一：導航 (左半部 Pane 0) =================
CMD1="ros2 launch navigation2_run real_launch.py"
tmux send-keys -t $SESSION_NAME:0.0 "$CMD1" C-m


# ================= 區塊二：定位 + Domain Bridge (右上 Pane 1) =================
# 如果有包含 domain bridge，利用 trap 確保 Ctrl+C 能同時殺掉兩個進程
if [ "$ROS_DOMAIN_ID" = "51" ] || [ "$ROS_DOMAIN_ID" = "61" ]; then
    echo "Launching domain bridge for sima-001 or sima-011"
    CMD2="bash -c \"trap 'kill 0' SIGINT; ros2 launch sima-localization-real robot_localization_yellow.launch.py & ros2 launch domain_bridge domain_bridge.launch.py & wait\""
else
    CMD2="ros2 launch sima-localization-real robot_localization_yellow.launch.py"
fi
tmux send-keys -t $SESSION_NAME:0.1 "$CMD2" C-m

sleep 5

# ================= 區塊三：sima-main 主程式 (右下 Pane 2) =================
# 將多個 ros2 run 包進一個 bash group 中執行
if [ "$ROS_DOMAIN_ID" = "51" ] || [ "$ROS_DOMAIN_ID" = "61" ]; then
    echo "Launching strategy node for sima-001 or sima-011"
    CMD3="bash -c \"trap 'kill 0' SIGINT; ros2 run sima-main system_check & ros2 run sima-main sima_navigator --ros-args --params-file ${PARAMS_FILE} & ros2 run sima-main sima_strategy --ros-args --params-file ${PARAMS_FILE} -p mission_type:=${MISSION_TYPE} & wait\""
else
    CMD3="bash -c \"trap 'kill 0' SIGINT; ros2 run sima-main system_check & ros2 run sima-main sima_navigator --ros-args --params-file ${PARAMS_FILE} & wait\""
fi
tmux send-keys -t $SESSION_NAME:0.2 "$CMD3" C-m

# ================= 進入 tmux 畫面 =================
tmux attach-session -t $SESSION_NAME
