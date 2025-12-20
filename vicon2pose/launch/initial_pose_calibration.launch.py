from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    vicon2pose = get_package_share_directory('vicon2pose')
    calibration_info_file = os.path.join(vicon2pose,'config','calibration_info.yaml')

    pose_calibration_node = Node(
            package='vicon2pose',
            executable='pose_calibration_node.py',
            output='screen',
            arguments=["-f",calibration_info_file]
        )
    launch_description = LaunchDescription()
    launch_description.add_action(pose_calibration_node)

    return launch_description
