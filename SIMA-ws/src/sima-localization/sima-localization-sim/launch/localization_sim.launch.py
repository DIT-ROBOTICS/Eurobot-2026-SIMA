from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sima-localization-sim',
            executable='global_sim_node',
            name='global_sim_node'
        ),
        Node(
            package='sima-localization-sim',
            executable='odom_sim_node',
            name='odom_sim_node'
        ),
        Node(
            package='sima-localization-sim',
            executable='robot_pose_node',
            name='robot_pose_node'
        ),
        # Static transform publisher from 'world' to 'map'
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '0', '0', 'world', 'map']
        )
    ])