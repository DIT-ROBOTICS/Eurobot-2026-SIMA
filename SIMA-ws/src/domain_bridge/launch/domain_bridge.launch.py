from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # Get the package share directory
    package_share_dir = get_package_share_directory('domain_bridge')

    ros_domain_id = os.getenv('ROS_DOMAIN_ID')
    config_file_path = "domain_bridge-config.yaml"
    if (ros_domain_id == '51'):
        config_file_path = os.path.join(package_share_dir, 'config', 'domain_bridge-config-A.yaml')
    elif (ros_domain_id == '61'):
        config_file_path = os.path.join(package_share_dir, 'config', 'domain_bridge-config-B.yaml')
    else:
        config_file_path = os.path.join(package_share_dir, 'config', 'domain_bridge-config.yaml')
      
    config_arg = DeclareLaunchArgument(
        "config", 
        default_value=config_file_path,
        description="Path to YAML configuration"
    )

    from_domain_arg = DeclareLaunchArgument(
        "from_domain", description="Override any 'from_domain' entries in the YAML file", default_value=""
    )

    to_domain_arg = DeclareLaunchArgument(
        "to_domain", description="Override any 'to_domain' entries in the YAML file", default_value=""
    )

    wait_sub_arg = DeclareLaunchArgument(
        'wait_for_subscription',
        default_value='false',
        description="Will wait for an available subscription before bridging a topic. This overrides any value set in the YAML file."
    )
    wait_pub_arg = DeclareLaunchArgument(
        'wait_for_publisher',
        default_value='true',
        description=" Will wait for an available subscription before bridging a topic. This overrides any value set in the YAML file"
    )


    def createBridge(context):
        from_domain = LaunchConfiguration("from_domain").perform(context)
        to_domain   = LaunchConfiguration("to_domain").perform(context)
        config = LaunchConfiguration('config').perform(context)
        wait_sub = LaunchConfiguration('wait_for_subscription').perform(context)
        wait_pub = LaunchConfiguration('wait_for_publisher').perform(context)

        args = []

        if from_domain:
            args.append("--from")
            args.append(str(from_domain))

        if to_domain:
            args.append("--to")
            args.append(str(to_domain))

        if wait_pub.lower() == "false":
            args.append("--wait-for-publisher")
            args.append(wait_pub)
        elif wait_sub.lower() == "true":
            args.append("--wait-for-subscription")
            args.append(wait_sub)



        args.append(config)
        
        return [Node(
            package="domain_bridge",
            executable="domain_bridge",
            output='both',
            arguments=args
        )]
      

    domain_bridge_node = OpaqueFunction(function=createBridge)


    return LaunchDescription([
        config_arg,
        from_domain_arg,
        to_domain_arg,
        wait_pub_arg,
        wait_sub_arg,

        domain_bridge_node
    ])