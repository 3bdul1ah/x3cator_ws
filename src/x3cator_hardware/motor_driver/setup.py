from setuptools import setup

package_name = 'motor_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='a2053231@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver_canopen = motor_driver.motor_driver_canopen:main',
            'odometry = motor_driver.odometry:main', 
            'robot_tf_publisher = motor_driver.robot_tf_publisher:main',
        ],
    },
)
