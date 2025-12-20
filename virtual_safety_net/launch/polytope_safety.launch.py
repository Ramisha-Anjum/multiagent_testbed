#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = 'virtual_safety_net'
    pkg_share = get_package_share_directory(pkg_name)

    # Paths inside config/ folder
    agents_file   = os.path.join(pkg_share, 'config', 'agents.yaml')
    boundary_file = os.path.join(pkg_share, 'config', 'boundary.yaml')

    # Safety node
    safety_node = Node(
        package=pkg_name,
        executable='polytope_safety_node.py',   # match your install/executable name
        name='polytope_safety_node',
        output='screen',
        parameters=[
            {
                'agents_file': agents_file,
                'boundary_file': boundary_file,
                'boundary_epsilon': 0.05
            }
        ]
    )

    return LaunchDescription([safety_node])
