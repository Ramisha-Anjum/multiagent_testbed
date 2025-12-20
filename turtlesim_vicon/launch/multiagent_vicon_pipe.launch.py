import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # ------------------------------------------------------------
    # Load agents from virtual_safety_net/config/agents.yaml
    # ------------------------------------------------------------
    vsn_share = get_package_share_directory('virtual_safety_net')
    agents_yaml_path = os.path.join(vsn_share, 'config', 'agents.yaml')

    with open(agents_yaml_path, 'r') as f:
        agents_cfg = yaml.safe_load(f)

    agents = agents_cfg.get("agents", [])

    if not agents:
        raise RuntimeError("No agents found in virtual_safety_net/config/agents.yaml")

    # ------------------------------------------------------------
    # 1) Vicon receiver
    # ------------------------------------------------------------
    vicon_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('vicon_receiver'),
                'launch',
                'client.launch.py'
            )
        )
    )

    # ------------------------------------------------------------
    # 2) vicon2pose launch files for each agent
    #    topic: /vicon/<agent>/<agent>
    # ------------------------------------------------------------
    vicon_pose_nodes = []
    vicon2pose_dir = os.path.join(get_package_share_directory('vicon2pose'), 'launch')

    for agent in agents:

        launch_file = os.path.join(vicon2pose_dir, 'pose_publisher.launch.py')

        node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={
                'vicon_topic': f'/vicon/{agent}/{agent}',
                'robot_name': agent
            }.items()
        )

        vicon_pose_nodes.append(node)

    # ------------------------------------------------------------
    # 3) Virtual Safety Net
    # ------------------------------------------------------------
    vsn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('virtual_safety_net'),
                'launch',
                'polytope_safety.launch.py'
            )
        )
    )

    # # ------------------------------------------------------------
    # # 4) multiagent_velocity_control (your node)
    # # ------------------------------------------------------------
    # rms1_share = get_package_share_directory('robomaster_s1')
    # params_file = os.path.join(rms1_share, 'config', 'config.yaml')

    # multiagent_velocity_control = Node(
    #     package='robomaster_s1',
    #     executable='multiagent_velocity_control.py',
    #     output='screen',
    #     parameters=[params_file],
    # )
    sensor_node = Node(package='turtlesim_vicon', executable='relative_position_emulator', name='sensor')
    
    # ------------------------------------------------------------
    # Build LaunchDescription
    # ------------------------------------------------------------
    return LaunchDescription(
        [
            LogInfo(msg="Launching multi-agent VICON + Safety + Control pipeline..."),
            vicon_client,
            *vicon_pose_nodes,
             vsn_launch,
            # multiagent_velocity_control,
            sensor_node
        ]
    )
