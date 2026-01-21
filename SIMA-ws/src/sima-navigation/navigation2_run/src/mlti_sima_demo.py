#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def main():
    rclpy.init()

    # --- 1. 建立兩個導航員 (Navigator) ---
    # 這裡我們利用 BasicNavigator 的 namespace 參數
    # 注意：BasicNavigator 預設會找 /namespace/navigate_to_pose
    navigator1 = BasicNavigator(namespace='robot1')
    navigator2 = BasicNavigator(namespace='robot2')

    # 等待 Nav2 啟動完成
    # print("Waiting for Robot 1 Nav2...")
    # navigator1.waitUntilNav2Active() # 如果你的 nav2 已經在 spin，這步可以跳過或縮短
    # print("Waiting for Robot 2 Nav2...")
    # navigator2.waitUntilNav2Active()

    # --- 2. 設定 Robot 1 的目標點 (假設地圖原點是 0,0) ---
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator1.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 2.0
    goal_pose1.pose.position.y = 0.5
    goal_pose1.pose.orientation.w = 1.0

    # --- 3. 設定 Robot 2 的目標點 ---
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator2.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 2.0
    goal_pose2.pose.position.y = 1.0 # 讓它去稍微不同的地方
    goal_pose2.pose.orientation.w = 1.0

    print("Sending goals to both robots...")

    # --- 4. 同時發送指令 ---
    # goToPose 是非阻塞 (Non-blocking) 的，所以兩個指令會幾乎同時發出
    navigator1.goToPose(goal_pose1)
    navigator2.goToPose(goal_pose2)

    # --- 5. 監控迴圈 (Optional) ---
    while not navigator1.isTaskComplete() or not navigator2.isTaskComplete():
        # 這裡可以做一些監控，或者只是單純等待
        # 例如：檢查有沒有人失敗，或者計算剩餘距離
        pass

    # 取得結果
    result1 = navigator1.getResult()
    result2 = navigator2.getResult()
    
    if result1 == TaskResult.SUCCEEDED:
        print('Robot 1 reached goal!')
    if result2 == TaskResult.SUCCEEDED:
        print('Robot 2 reached goal!')

    rclpy.shutdown()

if __name__ == '__main__':
    main()