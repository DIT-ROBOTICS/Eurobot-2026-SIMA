import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml, ReplaceString

def generate_launch_description():
    # 1. 取得路徑
    pkg_nav2_run = get_package_share_directory('navigation2_run')
    
    # 2. 宣告參數
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock')

    # 3. 定義原始參數檔與地圖
    # 注意：這裡讀取你原本的 nav2_params_sima.yaml
    params_file = os.path.join(pkg_nav2_run, 'params', 'nav2_params_sima.yaml')
    map_file = os.path.join(pkg_nav2_run, 'maps', 'basic_map.yaml')
    
    # 指向我們剛寫好的 Bridge launch
    bridge_launch = os.path.join(pkg_nav2_run, 'launch', 'vl53_bridge_launch.py')
    
    # 指向你原本的 bringup launch
    bringup_launch = os.path.join(pkg_nav2_run, 'launch', 'bringup_launch.py')

    # 4. 設定 Nav2 參數替換規則 (Magic happens here!)
    # Note: <robot_namespace> placeholder in YAML is replaced by bringup_launch.py
    # using ReplaceString before RewrittenYaml processes param_rewrites
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'odom_topic': [namespace, '_odom'],
        'scan_topic': [namespace, '/scan'],
        'topic': [namespace, '/detected_obstacles'] # Costmap 的 observation source
    }

    # 使用 RewrittenYaml 產生新的暫存參數檔
    configured_params = RewrittenYaml(
        source_file=params_file,
        # root_key=namespace, # 參數會被搬移到 namespace 下 (重要!)
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # 5. 組合啟動動作
    # 注意：我們不需要在外層包 PushRosNamespace，因為 bringup_launch.py 
    # 內部已經有處理 use_namespace 的邏輯了。我們只需要傳入參數即可。
    
    # A. 啟動 Bridge (Level 1)
    bridge_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bridge_launch),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time
        }.items()
    )

    # B. 啟動原本的 Bringup (呼叫你原本的檔案)
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': 'True', # 告訴 bringup_launch 我們要用 namespace
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': configured_params, # 傳入動態修改後的參數
            'autostart': 'True',
            'use_composition': 'True',
            'use_respawn': 'False',
            'use_odometry_sim': 'False',
            # 這裡傳入 robot_pose_remap 是為了 localization_launch.py
            # 你的 localization_launch.py 裡面用它來 remap /odom/wheel
            'robot_pose_remap': [namespace, '/odom'] 
        }.items()
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        bridge_cmd,
        nav2_bringup_cmd
    ])