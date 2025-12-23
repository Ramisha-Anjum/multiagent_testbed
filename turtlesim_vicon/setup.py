from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlesim_vicon'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mission-control',
    maintainer_email='mission-control@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'relative_position_emulator = turtlesim_vicon.relative_position_emulator:main',
            'relative_position_bearing_emulator = turtlesim_vicon.relative_position_bearing_emulator:main',
            'turtle2posestamped = turtlesim_vicon.turtle2posestamped:main',
        ],
    },
)