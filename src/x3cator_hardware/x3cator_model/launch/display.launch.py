from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('x3cator_model')
    urdf_file = os.path.join(pkg_share, 'urdf', 'x3cator_model.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Uncomment and modify if you want to add static transforms later
    # rslidar_to_base_link = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='rslidar_to_base_link',
    #     arguments=['-0.42186', '0', '-0.264', '0', '0', '0', 'camera_link', 'base_link']
    # )

    # base_link_to_footprint = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_link_to_base_footprint',
    #     arguments=['0', '0', '-0.45616404', '0', '0', '0', 'base_link', 'base_footprint']
    # )

    # Return LaunchDescription with the nodes to launch
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        # rslidar_to_base_link,
        # base_link_to_footprint
    ])
