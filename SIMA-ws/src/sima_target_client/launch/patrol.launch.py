import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('sima_target_client')
    waypoints_config = os.path.join(pkg_share, 'config', 'waypoints.yaml')

    return LaunchDescription([
        Node(
            package='sima_target_client',
            executable='patrol_node_cpp',
            name='patrol_node',
            output='screen',
            parameters=[
                {'waypoints_file': waypoints_config}
            ]
        )
    ])