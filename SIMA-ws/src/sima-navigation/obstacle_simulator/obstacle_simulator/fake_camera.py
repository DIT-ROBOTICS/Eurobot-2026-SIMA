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

    # === Function to generate a solid square obstacle ===
    def create_square_obstacle(self, center_x, center_y, size_meters, resolution=0.02):
        """
        Generate all points that make up a square
        :param center_x: Center point X
        :param center_y: Center point Y
        :param size_meters: Side length (e.g., 0.3 means 30cm)
        :param resolution: Point density (draw a point every N meters)
        :return: List of points [[x,y,z], [x,y,z]...]
        """
        points = []
        half_size = size_meters / 2.0
        
        # Calculate start and end points
        start_x = center_x - half_size
        end_x = center_x + half_size
        start_y = center_y - half_size
        end_y = center_y + half_size
        
        # Double loop to fill this area
        curr_x = start_x
        while curr_x <= end_x:
            curr_y = start_y
            while curr_y <= end_y:
                points.append([curr_x, curr_y, 0.2]) # Set z height to 0.2
                curr_y += resolution
            curr_x += resolution
            
        return points

    def timer_callback(self):
        all_points = []

        # --- 1. Create first obstacle: 10cm box at (1.5, 0.5) ---
        box1 = self.create_square_obstacle(1.5, 1.1, 0.2)
        all_points.extend(box1)

        # --- 2. Create second obstacle: 20cm box at (1.5, 1.0) ---
        # box2 = self.create_square_obstacle(1.5, 1.0, 0.2)
        # all_points.extend(box2)

        # --- 3. (Optional) Draw a wall ---
        # Simple simulation of a wall: X from 3.0 to 3.0, Y from -1.0 to 1.0
        # wall_points = []
        # y = -1.0
        # while y <= 1.0:
        #     wall_points.append([3.0, y, 0.2])
        #     y += 0.02
        # all_points.extend(wall_points)


        # --- Below is the standard packing process ---
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.height = 1
        msg.width = len(all_points)
        
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