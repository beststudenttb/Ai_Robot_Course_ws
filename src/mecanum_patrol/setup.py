import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mecanum_patrol'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tb',
    maintainer_email='tb@todo.todo',
    description='ROS2 nodes for a Webots mecanum patrol robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mecanum_driver = mecanum_patrol.mecanum_driver_node:main',
            'keyboard_app = mecanum_patrol.keyboard_app_node:main',
            'perception = mecanum_patrol.perception_node:main',
            'decision = mecanum_patrol.decision_node:main',
            'mission = mecanum_patrol.mission_node:main',
            'logger = mecanum_patrol.logger_node:main',
        ],
    },
)
