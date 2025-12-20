#!usr/bin/python3


import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    num_pose = LaunchConfiguration('num_pose')
    fusion_method = LaunchConfiguration('fusion_method')
    num_pose_launch_arg = DeclareLaunchArgument(
        name='num_pose',
        default_value='5',
        description='Number of fake poses to publish'
    )
    fusion_method_launch_arg = DeclareLaunchArgument(
        name='fusion_method',
        default_value='man_opt',
        description='Pose fusion method. Can be either "man_opt" or "avg_tan_vec"'    
    )
    pose_publisher = Node(
        package='pose_fusion',
        executable='fake_pose_publisher.py',
        parameters=[{'num_pose':num_pose}]
    )
    pose_fusion = Node(
        package='pose_fusion',
        executable='pose_fusion_example.py',
        parameters=[{'fusion_method':fusion_method}],
        output='both'
    )
    pkg = get_package_share_directory('pose_fusion')
    config_file = os.path.join(pkg,'config','config.rviz')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d',config_file]
    )
    launch_description = LaunchDescription()
    launch_description.add_action(num_pose_launch_arg)
    launch_description.add_action(fusion_method_launch_arg)
    launch_description.add_action(pose_publisher)
    launch_description.add_action(pose_fusion)
    launch_description.add_action(rviz2)
    return launch_description
    
