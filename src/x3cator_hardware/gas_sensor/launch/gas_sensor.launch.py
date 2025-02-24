#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    publisher_node = Node(
        package='gas_sensor',
        executable='gas_sensor_publisher',
        name='gas_sensor_publisher',
        arguments=['--ros-args', '--log-level', 'warn']

    )
    
    subscriber_node = Node(
        package='gas_sensor',
        executable='gas_sensor_subscriber',
        name='gas_sensor_subscriber',
    )
    
    return LaunchDescription([
        publisher_node,
        subscriber_node
    ])
