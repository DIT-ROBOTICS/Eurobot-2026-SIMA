import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 定義參數
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace (e.g. robot1)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Bridge Node
    # Note: Topics use relative paths in C++ code, so they automatically become
    # /robot1/sensors/raw_ranges and /robot1/sensors/detected_obstacles
    # Frame IDs need namespace prefix for multi-robot: robot1_base_link, not base_link
    bridge_node = Node(
        package='navigation2_run',
        executable='vl53_bridge',
        name='vl53_bridge',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'trigger_distance': 0.5},
            {'robot_base_frame': [namespace, '_base_link']},
            {'global_frame': 'map'}
        ]
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        bridge_node
    ])