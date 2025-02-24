from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_gas_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Install launch files
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        
        # Install configuration files
        ('share/' + package_name + '/config', glob('config/*.rviz')),
        
        # Install resource files
        ('share/' + package_name + '/resource', glob('resource/*')),
        
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',  # Replace with your name
    maintainer_email='your_email@example.com',  # Replace with your email
    description='A ROS 2 package for TurtleBot3 gas mapping and SLAM integration',
    license='MIT',  # Replace with your chosen license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'virtual_sensor = turtlebot3_gas_mapping.virtual_gas_sensor:main',
            'slam_gdm_integration = turtlebot3_gas_mapping.slam_gdm_integration:main',
            'gas_concentration_mapper = turtlebot3_gas_mapping.gas_concentration_mapper:main',
        ],
    },
)