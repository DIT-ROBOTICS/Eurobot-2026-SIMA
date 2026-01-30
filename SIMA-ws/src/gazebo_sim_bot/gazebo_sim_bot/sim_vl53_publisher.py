#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class SimVL53Publisher(Node):
    def __init__(self):
        super().__init__('sim_vl53_publisher')

        self.declare_parameter('robot_name', 'sima1')
        self.robot_name = self.get_parameter('robot_name').value

        prefix = f'/{self.robot_name}'
        
        # subscribe LaserScan topics send from gazebo (see the urdf files)
        # note that we use qos_profile_sensor_data to match gazebo's best effort policy
        self.create_subscription(LaserScan, f'{prefix}/vl53_left', self.cb_left, qos_profile_sensor_data)
        self.create_subscription(LaserScan, f'{prefix}/vl53_center', self.cb_center, qos_profile_sensor_data)
        self.create_subscription(LaserScan, f'{prefix}/vl53_right', self.cb_right, qos_profile_sensor_data)
        
        # Publish array to Bridge
        self.pub = self.create_publisher(Float32MultiArray, f'{prefix}/sensors/raw_ranges', 10)
        
        # Default value is 4.0 (representing no obstacle)
        self.ranges = [4.0, 4.0, 4.0] 

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, f'{prefix}/ground_truth_pose', self.odom_callback, 10)
        
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info(f"Sim VL53 Adapter Started for {self.robot_name}")
    
    def odom_callback(self, msg):
        t = TransformStamped()

        # Fill in the header
        t.header.stamp = msg.header.stamp
        # To work with Nav2, we force it to be robot_name/odom (e.g., sima1/odom)
        t.header.frame_id = f'{self.robot_name}/odom' 
        t.child_frame_id = f'{self.robot_name}/base_link'

        # Fill in the translation (directly copied from Odom message)
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Fill in the rotation
        t.transform.rotation = msg.pose.pose.orientation

        # Publish TF
        self.tf_broadcaster.sendTransform(t)

    # Callback functions: handle LaserScan
    def cb_left(self, msg): 
        self.process_scan(msg, 0)
    
    def cb_center(self, msg): 
        self.process_scan(msg, 1)
        
    def cb_right(self, msg): 
        self.process_scan(msg, 2)

    # General processing function
    def process_scan(self, msg, index):
        # Since we set samples=1 in URDF, msg.ranges has only one value
        # For safety, we check the length first
        if len(msg.ranges) > 0:
            dist = msg.ranges[0]
            # Handle infinite (inf) values: Gazebo returns inf when it doesn't detect anything
            # We convert it to 4.0 (or a value greater than your trigger distance in vl53_bridge)
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