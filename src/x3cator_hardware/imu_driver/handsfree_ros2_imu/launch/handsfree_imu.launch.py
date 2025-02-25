import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    imu_type_arg = DeclareLaunchArgument(
        'imu_type',
        default_value='a9',
        description='IMU type [a9, b9, b6]'
    )

    imu_type = LaunchConfiguration('imu_type')

    imu_node = Node(
        package='handsfree_ros2_imu',
        executable=[f'hfi_{imu_type.perform(None)}_ros2.py'],
        name='imu',
        output='screen',
        parameters=[{
            'port': '/dev/HFRobotIMU',
            'gra_normalization': imu_type.perform(None) == 'a9'
        }]
    )

    return LaunchDescription([
        imu_type_arg,
        imu_node
    ])
