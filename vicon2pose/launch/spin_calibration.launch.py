from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pose_calibration_node = Node(
            package='vicon2pose',
            executable='spin_calibration_node.py',
            output='screen',
        )
    launch_description = LaunchDescription()
    launch_description.add_action(pose_calibration_node)

    return launch_description
