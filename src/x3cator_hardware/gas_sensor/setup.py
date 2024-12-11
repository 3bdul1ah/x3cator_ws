from setuptools import setup

package_name = 'gas_sensor'

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
            'gas_sensor_publisher = gas_sensor.gas_sensor_publisher:main',
            'gas_sensor_subscriber = gas_sensor.gas_sensor_subscriber:main',
            'fake_gas_sensor_publisher = gas_sensor.fake_gas_sensor_publisher:main',

        ],
    },
)
