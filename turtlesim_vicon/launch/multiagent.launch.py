#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml

def generate_launch_description():
    pkg_multiagent = get_package_share_directory('turtlesim_vicon')
    config_file = os.path.join(pkg_multiagent, 'config', 'config_sim.yaml')

    with open(config_file, 'r') as f:
        data = yaml.safe_load(f)

    net = data['network']
    num_agents = int(net['num_agents'])
    positions = [(float(p['x']), float(p['y'])) for p in net['initial_position']]

    # sanity check
    if len(positions) < num_agents:
        raise ValueError("initial_position must have at least 'num_agents' entries")

    turtlesim_node = Node(package='turtlesim', executable='turtlesim_node', name='sim')

    ld = LaunchDescription()
    ld.add_action(turtlesim_node)

    # Spawn turtles 2..N (turtle1 already exists)
    # Stagger calls a bit to avoid hammering /spawn
    for i in range(2, num_agents + 1):
        x, y = positions[i - 1]  # agent i -> positions[i-1]
        name = f"turtle{i}"
        spawn_cmd = [
            'ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
            f"{{x: {x}, y: {y}, theta: 0.0, name: '{name}'}}"
        ]
        # delay each spawn a bit more than the previous one
        spawn_action = TimerAction(
            period=0.3 * (i - 1),  # 0.3s, 0.6s, ...
            actions=[ExecuteProcess(cmd=spawn_cmd, output='screen')]
        )
        ld.add_action(RegisterEventHandler(
            OnProcessStart(target_action=turtlesim_node, on_start=[spawn_action])
        ))

    
    t2p_node = Node(package='turtlesim_vicon', executable='turtle2posestamped', name='t2p')
    sensor_node = Node(package='turtlesim_vicon', executable='relative_position_bearing_emulator', name='sensor')
    #sensor_node = Node(package='turtlesim_vicon', executable='relative_position_emulator', name='sensor')
    #algorithm_node = Node(package='multiagent_est_con', executable='algorithm.py', name= 'algorithm')
    ld.add_action(t2p_node)
    ld.add_action(sensor_node)
    #ld.add_action(algorithm_node)
    return ld
