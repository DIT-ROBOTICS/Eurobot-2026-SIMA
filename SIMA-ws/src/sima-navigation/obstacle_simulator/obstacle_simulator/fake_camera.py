# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2, PointField
# import struct

# class FakeCamera(Node):
#     def __init__(self):
#         super().__init__('fake_camera_node')
#         # 發布到 /detected_obstacles topic
#         self.publisher_ = self.create_publisher(PointCloud2, '/detected_obstacles', 10)
#         # 每 0.5 秒發布一次
#         self.timer = self.create_timer(0.5, self.timer_callback)
#         self.get_logger().info('Fake Camera Simulator Started.')

#     def timer_callback(self):
#         # --- 定義多個障礙物座標 (Map Frame) ---
#         # 格式：[x, y, z]
#         obstacles = [
#             [1.0, 1.0, 0.0],   # 障礙物 A (左邊一點)
#             [1.5, 0.5, 0.0],  # 障礙物 B (右邊一點)
#             # 你可以繼續往下加... [3.0, 0.0, 0.2]
#         ]

#         # 建立 PointCloud2 訊息
#         msg = PointCloud2()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = 'map' 
        
#         msg.height = 1
#         msg.width = len(obstacles)  # [關鍵] 寬度等於點的數量
        
#         msg.fields = [
#             PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#             PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#             PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
#         ]
#         msg.is_bigendian = False
#         msg.point_step = 12
#         msg.row_step = 12 * len(obstacles) # 一列的總 byte 數
#         msg.is_dense = True
        
#         # --- [關鍵修改] 打包所有點的數據 ---
#         data_buffer = bytearray()
#         for p in obstacles:
#             # p[0]=x, p[1]=y, p[2]=z
#             data_buffer += struct.pack('fff', p[0], p[1], p[2])
            
#         msg.data = data_buffer

#         self.publisher_.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = FakeCamera()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()



import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
import math

class FakeCamera(Node):
    def __init__(self):
        super().__init__('fake_camera_node')
        self.publisher_ = self.create_publisher(PointCloud2, '/detected_obstacles', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('Fake Camera Simulator Started.')

    # === [新增] 產生一個實心正方形的函式 ===
    def create_square_obstacle(self, center_x, center_y, size_meters, resolution=0.02):
        """
        產生組成正方形的所有點
        :param center_x: 中心點 X
        :param center_y: 中心點 Y
        :param size_meters: 邊長 (例如 0.3 代表 30公分)
        :param resolution: 點的密度 (每隔幾公尺畫一點)
        :return: 點的列表 [[x,y,z], [x,y,z]...]
        """
        points = []
        half_size = size_meters / 2.0
        
        # 計算起點和終點
        start_x = center_x - half_size
        end_x = center_x + half_size
        start_y = center_y - half_size
        end_y = center_y + half_size
        
        # 雙層迴圈填滿這個區域
        curr_x = start_x
        while curr_x <= end_x:
            curr_y = start_y
            while curr_y <= end_y:
                points.append([curr_x, curr_y, 0.2]) # z高度設為0.2
                curr_y += resolution
            curr_x += resolution
            
        return points

    def timer_callback(self):
        all_points = []

        # --- 1. 建立第一個障礙物：在 (2.0, 0.5) 的 30cm 箱子 ---
        box1 = self.create_square_obstacle(1.0, 0.7, 0.1)
        all_points.extend(box1)

        # --- 2. 建立第二個障礙物：在 (2.0, -0.5) 的 20cm 箱子 ---
        # box2 = self.create_square_obstacle(2.0, -0.5, 0.2)
        # all_points.extend(box2)

        # --- 3. (選用) 甚至可以畫一條牆壁 ---
        # 這裡簡單模擬一條牆壁：X 從 3.0 到 3.0，Y 從 -1.0 到 1.0
        # wall_points = []
        # y = -1.0
        # while y <= 1.0:
        #     wall_points.append([3.0, y, 0.2])
        #     y += 0.02
        # all_points.extend(wall_points)


        # --- 以下是標準打包流程 (跟之前一樣) ---
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.height = 1
        msg.width = len(all_points) # 這裡會自動變多
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(all_points)
        msg.is_dense = True
        
        data_buffer = bytearray()
        for p in all_points:
            data_buffer += struct.pack('fff', p[0], p[1], p[2])
            
        msg.data = data_buffer

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()