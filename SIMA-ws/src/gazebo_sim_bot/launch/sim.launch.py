# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# import xacro

# def generate_launch_description():
#     # 1. 指定機器人描述檔的位置
#     pkg_name = 'sim_bot'
#     file_subpath = 'urdf/my_robot.urdf.xacro'
    
#     # 透過 xacro 解析檔案
#     xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
#     robot_description_raw = xacro.process_file(xacro_file).toxml()

#     # 2. 設定 Gazebo
#     # 使用 gazebo_ros 內建的 launch 檔來啟動世界
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([os.path.join(
#             get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
#     )

#     # 3. 生成機器人 (Spawn Entity)
#     # 這個節點負責把 URDF 送進 Gazebo 裡
#     spawn_entity = Node(
#         package='gazebo_ros', 
#         executable='spawn_entity.py',
#         arguments=['-topic', 'robot_description',
#                    '-entity', 'my_bot'], # 在 Gazebo 裡的名字
#         output='screen'
#     )

#     # 4. Robot State Publisher
#     # 這個節點負責發布機器人的結構 TF (座標轉換)
#     node_robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[{'robot_description': robot_description_raw}]
#     )

#     return LaunchDescription([
#         gazebo,
#         node_robot_state_publisher,
#         spawn_entity,
#     ])


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command # <--- 新增這個
from launch_ros.actions import Node
# 移除 import xacro  <--- 這一行刪掉

def generate_launch_description():
    pkg_name = 'gazebo_sim_bot' # 注意：我看你的錯誤訊息，你的 package 名字好像是 gazebo_sim_bot？如果是的話這裡要改對
    file_subpath = 'urdf/my_robot.urdf.xacro'
    
    world_file_name = 'eurobot_only_walls.world'
    world_path = os.path.join(get_package_share_directory(pkg_name), 'world', world_file_name)

    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    
    # --- 修改重點開始 ---
    # 不使用 python 直接解析，而是改用 Command 指令讓系統去執行 xacro
    robot_description_raw = Command(['xacro ', xacro_file])
    # --- 修改重點結束 ---

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_bot',
                    '-x', '0.5',
                    '-y', '0.5',
                    '-z', '0.05'],
        output='screen'
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        # 這裡的 robot_description_raw 變成了一個 Command 物件，Launch 系統會在執行時自動解析它
        parameters=[{'robot_description': robot_description_raw}]
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])