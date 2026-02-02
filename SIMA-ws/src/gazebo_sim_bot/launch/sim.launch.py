import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'gazebo_sim_bot'
    bridge_config_path = os.path.join(get_package_share_directory(pkg_name), 'config', 'bridge_config.yaml')
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'my_robot.urdf.xacro')
    
    robot_name_1 = 'sima1'
    robot_name_2 = 'sima2'
    
    world_file_name = 'eurobot_only_walls.world'
    world_path = os.path.join(get_package_share_directory(pkg_name), 'world', world_file_name)
    
    robot_description_raw_1 = Command(['xacro ', xacro_file, ' robot_name:=', robot_name_1])
    robot_description_raw_2 = Command(['xacro ', xacro_file, ' robot_name:=', robot_name_2])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items()
    )

    ###################################################################################
    # Spawn Robot Entities
    ###################################################################################

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', f'{robot_name_1}/robot_description',
                   '-entity', robot_name_1,
                   '-robot_namespace', robot_name_1,
                   '-x', '0.5',
                   '-y', '0.5',
                   '-z', '0.05'],
        output='screen'
    )

    spawn_entity_2 = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', f'{robot_name_2}/robot_description',
                   '-entity', robot_name_2,
                   '-robot_namespace', robot_name_2,
                   '-x', '1.0',
                   '-y', '0.5',
                   '-z', '0.05'],
        output='screen'
    )

    ###################################################################################
    # Robot State Publisher Nodes
    ###################################################################################

    node_robot_state_publisher_1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        # Here, the robot_description_raw becomes a Command object, which the Launch system will automatically parse at runtime
        parameters=[{'robot_description': robot_description_raw_1, 'frame_prefix': f'{robot_name_1}/'}],  
        remappings=[('robot_description', f'/{robot_name_1}/robot_description')]
    )

    node_robot_state_publisher_2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw_2, 'frame_prefix': f'{robot_name_2}/'}],  
        remappings=[('robot_description', f'/{robot_name_2}/robot_description')]
    )

    ###################################################################################
    # Sim VL53 Publisher Nodes
    ###################################################################################

    sim_vl53_publisher_node_1 = Node(
        package=pkg_name,
        executable='sim_vl53_publisher',
        name='sim_vl53_publisher_1',
        output='screen',
        parameters=[{'robot_name': robot_name_1}],
    )

    sim_vl53_publisher_node_2 = Node(
        package=pkg_name,
        executable='sim_vl53_publisher',
        name='sim_vl53_publisher_2',
        output='screen',
        parameters=[{'robot_name': robot_name_2}],
    )

    ###################################################################################
    # Domain Bridge Node
    ###################################################################################

    domain_bridge_node = Node(
        package='domain_bridge',
        executable='domain_bridge',
        name='domain_bridge',
        output='screen',
        arguments=[bridge_config_path],
    )

    # For gazebo simulation, we just publish a static transform from map to odom
    # However it's difficult to send it to different domains and prune the robot_name prefix
    # So we comment it out here and publish it in navigation2_run package instead
    # map_to_odom_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', f'{robot_name}/odom'],
    #     output='screen',
    #     namespace=robot_name,
    # )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher_1,
        node_robot_state_publisher_2,
        spawn_entity,
        spawn_entity_2,
        sim_vl53_publisher_node_1,
        sim_vl53_publisher_node_2,
        domain_bridge_node,
    ])