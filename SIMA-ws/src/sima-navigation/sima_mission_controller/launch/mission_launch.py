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
                # If you want to make waypoints as parameters in the future, you can add them here
                # {'waypoints_x': [1.0, 2.0]},
            ]
        )
    ])