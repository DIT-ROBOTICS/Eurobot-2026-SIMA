from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_localization_dir = get_package_share_directory('sima-localization-real')
    parameters_file_dir = os.path.join(robot_localization_dir, 'config')

    parameters_file_name = 'dual_ekf-real.yaml'
    ros_domain_id = os.getenv('ROS_DOMAIN_ID')

    if ros_domain_id == '51':
        parameters_file_name = 'dual_ekf-real-1-yellow.yaml'
    elif ros_domain_id == '52':
        parameters_file_name = 'dual_ekf-real-2-yellow.yaml'
    elif ros_domain_id == '53':
        parameters_file_name = 'dual_ekf-real-3-yellow.yaml'
    elif ros_domain_id == '54':
        parameters_file_name = 'dual_ekf-real-4-yellow.yaml'
    elif ros_domain_id == '61':
        parameters_file_name = 'dual_ekf-real-11-yellow.yaml'
    elif ros_domain_id == '62':
        parameters_file_name = 'dual_ekf-real-12-yellow.yaml'
    elif ros_domain_id == '63':
        parameters_file_name = 'dual_ekf-real-13-yellow.yaml'
    elif ros_domain_id == '64':
        parameters_file_name = 'dual_ekf-real-14-yellow.yaml'
    else:
        parameters_file_name = 'dual_ekf-real.yaml'
    
    parameters_file_path = os.path.join(parameters_file_dir, parameters_file_name)
    
    os.environ['FILE_PATH'] = str(parameters_file_dir)
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'output_final_position',
            default_value='false'),
        launch.actions.DeclareLaunchArgument(
            'output_location',
	    default_value='~/dual_ekf_navsat_example_debug.txt'),
	
    launch_ros.actions.Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_odom',
	        output='screen',
            parameters=[parameters_file_path],         
           ),
    launch_ros.actions.Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_map',
	        output='screen',
            parameters=[parameters_file_path],
           ),
    # launch_ros.actions.Node(
    #         package='micro_ros_agent', 
    #         executable='micro_ros_agent', 
    #         name='micro_ros_agent_serial',
    #         arguments=['serial', '--dev', '/dev/ttyAMA0', '-b', '1000000'],
    #         output='screen',
    #         ),
    launch_ros.actions.Node(
            package='sima-localization-real', 
            executable='sima-localization-odom-bridge_node', 
            name='sima_localization_odom_bridge_node',
            output='screen',
            )
])