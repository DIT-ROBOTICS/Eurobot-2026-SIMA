import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('node_health_monitor')
    config_path = os.path.join(pkg_dir, 'config', 'target_nodes.yaml')

    return LaunchDescription([
        Node(
            package='node_health_monitor',
            executable='supervisor',
            name='node_health_monitor',
            output='screen',
            parameters=[
                {'config_file': config_path}
            ]
        )
    ])