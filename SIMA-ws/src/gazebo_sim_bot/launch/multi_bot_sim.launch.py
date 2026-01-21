import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import Command

def generate_launch_description():
    # ========================================================================
    # 1. 設定變數 (請修改這裡的 package 名稱)
    # ========================================================================
    pkg_name = 'gazebo_sim_bot'  # <--- 請確認這裡改成你的 package 名稱
    urdf_file_name = 'my_robot.urdf.xacro' # <--- 請確認這是你的 xacro 檔名

    pkg_share = get_package_share_directory(pkg_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file_name) # 假設 urdf 放在 urdf/ 資料夾下

    world_file_name = 'eurobot_only_walls.world'
    world_path = os.path.join(get_package_share_directory(pkg_name), 'world', world_file_name)
    
    # Gazebo 的啟動檔 (來自 gazebo_ros package)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )


    # ========================================================================
    # 2. 定義生成單一機器人的函式
    # ========================================================================
    def create_robot_instance(name, x_pos, y_pos):
        # 這裡會生成 "robot1_" 這樣的字串傳給 xacro
        # 你的 xacro 接收 robot_name 參數，並加上 _ 作為 prefix
        prefix = name + "_" 
        
        # 使用 xacro 指令動態生成 URDF XML
        # Command 會在執行時呼叫 xacro 並傳入參數
        robot_description_content = Command(
            ['xacro ', urdf_path, ' robot_name:=', prefix]
        )
        
        return GroupAction([
            # A. 設定命名空間 (例如 /robot1)
            # 這個 group 下的所有 Node topic 都會變成 /robot1/xxx
            PushRosNamespace(name),

            # B. 機器人狀態發布 (Robot State Publisher)
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_description': robot_description_content
                }]
            ),

            # C. 在 Gazebo 生成實體 (Spawn Entity)
            # 注意：這裡不需要 PushRosNamespace，因為它是對 Gazebo 全域服務呼叫
            # 但我們透過 -robot_namespace 參數告訴 Gazebo 這個模型的 plugin 要聽誰的
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description', # 讀取上面 RSP 發布的 description
                    '-entity', name,              # Gazebo 裡的模型名稱 (robot1)
                    '-x', str(x_pos), '-y', str(y_pos), '-z', '0.1',
                    '-robot_namespace', name      # 關鍵：告訴 plugin 它的 namespace 是 robot1
                ],
                output='screen'
            ),

            Node(
                package=pkg_name, # 確認這是你的 package 名稱 (例如 gazebo_sim_bot)
                executable='sim_vl53_publisher.py', # 對應 CMakeLists install 的檔名
                name='vl53_adapter',
                output='screen',
                parameters=[{'use_sim_time': True}] # 重要：讓它跟 Gazebo 時間同步
            )
        ])
    
    # Helper function to create static TF for map->odom (must be outside namespace)
    def create_static_tf(name):
        prefix = name + "_"
        return Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'map_to_{name}_odom_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', prefix + 'odom'],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )

    # ========================================================================
    # 3. 建立實例
    # ========================================================================
    # 機器人 1 號：位置 (0.5, 0.5)
    robot1_cmd = create_robot_instance('robot1', 0.5, 0.5)
    
    # 機器人 2 號：位置 (1.0, 0.5)
    robot2_cmd = create_robot_instance('robot2', 1.0, 0.5)

    # ========================================================================
    # 4. 回傳 Launch Description
    # ========================================================================
    ld = LaunchDescription()
    
    # 啟動 Gazebo
    ld.add_action(gazebo_launch)
    
    # 啟動兩隻機器人
    ld.add_action(robot1_cmd)
    ld.add_action(robot2_cmd)
    
    # Add static TF transforms (OUTSIDE namespaces to publish on global /tf_static)
    ld.add_action(create_static_tf('robot1'))
    ld.add_action(create_static_tf('robot2'))

    return ld