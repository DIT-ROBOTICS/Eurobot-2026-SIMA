#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan  # <--- 改用 LaserScan
from std_msgs.msg import Float32MultiArray
from rclpy.qos import qos_profile_sensor_data # 必須使用這組 QoS

class SimVL53Publisher(Node):
    def __init__(self):
        super().__init__('sim_vl53_publisher')
        
        # 1. 訂閱 Gazebo 的 LaserScan Topic
        # 注意：這裡使用 qos_profile_sensor_data 是為了配合 Gazebo 的 Best Effort 發布
        self.create_subscription(LaserScan, '/sim/vl53_left', self.cb_left, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/sim/vl53_center', self.cb_center, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/sim/vl53_right', self.cb_right, qos_profile_sensor_data)
        
        # 2. 發布給 Bridge 的陣列
        self.pub = self.create_publisher(Float32MultiArray, '/sensors/raw_ranges', 10)
        
        # 預設值給 4.0 (代表沒障礙物)
        self.ranges = [4.0, 4.0, 4.0] 
        
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("Sim VL53 Adapter (LaserScan Mode) Started.")

    # 回呼函式：處理 LaserScan
    def cb_left(self, msg): 
        self.process_scan(msg, 0)
    
    def cb_center(self, msg): 
        self.process_scan(msg, 1)
        
    def cb_right(self, msg): 
        self.process_scan(msg, 2)

    # 通用處理函式
    def process_scan(self, msg, index):
        # 因為我們在 URDF 設 samples=1，所以 msg.ranges 只有一個值
        # 為了安全，先檢查長度
        if len(msg.ranges) > 0:
            dist = msg.ranges[0]
            # 處理無限大 (inf) 的情況：Gazebo 掃不到東西會回傳 inf
            # 我們把它轉成 4.0 (或大於你的 trigger distance 的值)
            if dist == float('inf'):
                self.ranges[index] = 4.0
            else:
                self.ranges[index] = dist

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = self.ranges
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimVL53Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()