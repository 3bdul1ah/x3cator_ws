from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os

def generate_launch_description():
    pkg_turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    pkg_slam_toolbox = FindPackageShare('slam_toolbox')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Gazebo simulation launch
        Node(
            package='turtlebot3_gazebo',
            executable='turtlebot3_world.launch.py',
            name='gazebo',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # SLAM tool integration
        Node(
            package='slam_toolbox',
            executable='online_async_launch.py',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Gas concentration mapper
        Node(
            package='turtlebot3_gas_mapping',
            executable='gas_concentration_mapper',
            name='gas_concentration_mapper',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
