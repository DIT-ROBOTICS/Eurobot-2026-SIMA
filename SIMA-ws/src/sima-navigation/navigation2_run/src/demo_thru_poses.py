import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String # 用來切換 controller
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped    # for ekf initial pose

def set_ekf_pose(node, x, y, yaw=0.0):
    """
    發送 /set_pose 指令給 robot_localization (EKF)
    強制重置機器人位置
    """
    # 建立一個臨時 Publisher
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initial_pose', 10)
    
    # 等待 Publisher 連線建立
    # (如果不等，第一則訊息常會寄丟)
    import time
    time.sleep(1.0) 

    msg = PoseWithCovarianceStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'map' # 或是 'odom'，看你的 EKF 設定，通常 map 比較保險

    # 設定座標
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    
    # 設定角度 (Yaw 轉 Quaternion)
    import math
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

    # 發布！
    print(f"正在重置 EKF 位置到: x={x}, y={y}")
    pub.publish(msg)
    
    # 給 EKF 一點時間反應
    time.sleep(0.5)
    node.destroy_publisher(pub) # 用完就丟

def create_pose(navigator, x, y, w=1.0):
    """
    快速建立 PoseStamped 的輔助函式
    """
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = w
    return pose

def main():
    # 1. 初始化
    rclpy.init()

    # 建立一個臨時 Node 用來發送 EKF 初始位置
    # node = rclpy.create_node('pose_setter')
    # set_ekf_pose(node, 0.5, 1.5, yaw=0.0)
    
    # 建立一個臨時 Node 用來發送 Controller 切換訊號
    # 因為 BasicNavigator 本身沒辦法發送自定義 topic，所以我們自己建一個 node
    selector_node = rclpy.create_node('controller_selector_node')
    latching_qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL, # 關鍵！要跟 Nav2 一樣是持久的
        reliability=ReliabilityPolicy.RELIABLE
    )

    controller_pub = selector_node.create_publisher(String, '/controller_type_thru', latching_qos)
    
    # 啟動 Nav2 指揮官
    navigator = BasicNavigator()
    
    # print("等待 Nav2 系統啟動...")
    # navigator.waitUntilNav2Active()
    # print("Nav2 已就緒！")

    initial_pose = create_pose(navigator, 1.0, 1.5, w=1.0) # 改成真實的出發座標
    navigator.setInitialPose(initial_pose) 

    # ==========================================
    # 2. 切換 Controller 為 "Diff"
    # ==========================================
    # 這是為了避開你在 XML 裡預設的 "Slow" (全向輪控制器)
    # 我們要用 "Diff" (差動輪控制器)，因為它有你寫好的避障邏輯
    msg = String()
    msg.data = "Diff"
    
    print("Switching controller: Diff ...")
    # 多發幾次確保 Behavior Tree 有收到 (網路傳輸有時會掉包)
    for _ in range(5):
        controller_pub.publish(msg)
        time.sleep(0.1) # 休息一下再發
    
    # ==========================================
    # 3. 設定路徑點 (Waypoints)
    # ==========================================
    # 這裡你可以根據你的地圖座標來修改
    # 假設場景：從起點出發，繞一個半圓，最後停在遠處
    
    waypoints = []
    
    # 點 1: 假設要先經過場地左側
    waypoints.append(create_pose(navigator, 0.5, 0.7))
    
    # 點 2: 然後往中間走
    waypoints.append(create_pose(navigator, 2.0, 0.7))
    
    # 點 3 (終點): 最後停在右上角
    # 注意：最後一點的角度很重要，機器人最後會轉向這個角度
    waypoints.append(create_pose(navigator, 2.0, 1.2, w=1.0))

    # ==========================================
    # 4. 發送指令 (Action Call)
    # ==========================================
    print(f"start nav through poses, total: {len(waypoints)} points...")
    navigator.goThroughPoses(waypoints)

    # ==========================================
    # 5. 監控任務
    # ==========================================
    i = 0
    while not navigator.isTaskComplete():
        # 每秒印一次回饋
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(f'Distance to next point: {feedback.distance_remaining:.2f} meters')
            
            # [進階技巧]
            # 如果你在這裡發現 feedback.distance_remaining 很久沒變
            # 代表機器人卡住了，你可以 navigator.cancelTask()

    # ==========================================
    # 6. 結束處理
    # ==========================================
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Navigation succeeded!')
    elif result == TaskResult.CANCELED:
        print('Navigation was canceled.')
    elif result == TaskResult.FAILED:
        print('Navigation failed!')
    selector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()