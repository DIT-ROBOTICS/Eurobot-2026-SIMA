from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sima_mission_controller',
            executable='sima_navigator',
            name='sima_navigator_node',
            output='screen',
            parameters=[
                # 如果未來想把 Waypoints 變成參數，可以加在這裡
                # {'waypoints_x': [1.0, 2.0]},
            ]
        )
    ])