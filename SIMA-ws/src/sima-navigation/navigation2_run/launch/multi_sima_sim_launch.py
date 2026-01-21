import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_gazebo_sim = get_package_share_directory('gazebo_sim_bot')
    pkg_nav2_run = get_package_share_directory('navigation2_run')

    # 1. 啟動 Gazebo 物理環境 
    # (這會呼叫你之前寫好的 multi_bot_sim.launch.py，負責 spawn robot, python adapter, static tf)
    gazebo_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_sim, 'launch', 'multi_bot_sim.launch.py')
        )
    )

    # 2. 定義啟動軟體 (Nav2 + Bridge) 的函式
    def launch_robot_software(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_run, 'launch', 'sima_launch.py')
            ),
            launch_arguments={
                'namespace': name,
                'use_sim_time': 'True'
            }.items()
        )

    # 3. 組合
    ld = LaunchDescription()
    
    # 啟動 Gazebo 環境
    ld.add_action(gazebo_sim_launch)
    
    # 延遲 5 秒後啟動 robot1 的軟體 (讓 Gazebo 先準備好)
    ld.add_action(TimerAction(
        period=5.0,
        actions=[launch_robot_software('robot1')]
    ))

    # 延遲 8 秒後啟動 robot2 的軟體 (錯開 CPU 負載)
    ld.add_action(TimerAction(
        period=8.0,
        actions=[launch_robot_software('robot2')]
    ))

    return ld