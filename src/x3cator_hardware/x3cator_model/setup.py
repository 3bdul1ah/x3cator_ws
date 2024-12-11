from setuptools import setup
import os
from glob import glob

package_name = 'x3cator_model'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),

        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*')),

        (os.path.join('share', package_name, 'meshes'),
         glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO@email.com',
    description='URDF Description package for x3cator_model robot',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
