import os
import sys

from ament_index_python.packages import get_package_share_directory # type: ignore

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription # type: ignore
from launch.conditions import IfCondition # type: ignore
from launch.substitutions import LaunchConfiguration, PythonExpression # type: ignore
from launch_ros.actions import Node # type: ignore
from launch.launch_description_sources import PythonLaunchDescriptionSource # type: ignore


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('sima-main')
    launch_dir = os.path.join(pkg_dir, 'launch')
    ros_domain_id = os.getenv('ROS_DOMAIN_ID')

    params_file_name = 'main_params_sima_test.yaml'

    # If needed different params file, change to the following logic
    # if ros_domain_id == '50':  # sima-test
    #     params_file_name = 'main_params_sima_test.yaml'
    # elif ros_domain_id == '51':  # sima-001
    #     params_file_name = 'main_params_sima_001.yaml'

    # Create the launch configuration variables
    params_file = LaunchConfiguration('params_file')

    # Declare the launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value = os.path.join(pkg_dir, 'params', params_file_name),
        description='Path to the parameters file for main'
    )

    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('sima-localization-real'), 'launch', 'robot_localization.launch.py')),
    )
    
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('navigation2_run'), 'launch', 'real_launch.py')),
    )
    
    domain_bridge_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('sima-package_launching'), 'launch', 'domain_bridge_test.launch.py')),
    )

    system_check_cmd = Node(
        package='sima-main',
        executable='system_check',
        name='system_check',
        output='screen',
    )

    sima_navigator_cmd = Node(
        package='sima-main',
        executable='sima_navigator',
        name='sima_navigator',
        output='screen',
        parameters=[params_file]
    )

    ld = LaunchDescription()

    ld.add_action(declare_params_file_cmd)

    ld.add_action(localization_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(domain_bridge_cmd)
    ld.add_action(system_check_cmd)
    ld.add_action(sima_navigator_cmd)

    return ld