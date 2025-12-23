#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Node: relative_neighbors_ps_node
# Package: multiagent_est_con
#
# Subscribes to geometry_msgs/PoseStamped for each agent:
#   topic: <pose_topic_template>.format(i), default: '/agent{}/pose'
# Computes y_{i,j} = p_j - p_i in world frame (z assumed 0)
# Publishes:
#   topic: turtle{i}/relative_neighbors (matching your existing topics)
#   type : relative_position_msg/RelativeNeighbors

import os
import yaml
from typing import Dict, Tuple, List, Set
import numpy as np
import math


import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Vector3

from relative_position_msg.msg import RelativeNeighbors, RelativeMeasurement


class RelativeNeighborsPSNode(Node):
    def __init__(self):
        super().__init__('relative_neighbors_ps_node')

        # Parameters
        self.declare_parameter('config_file', 'config_sim.yaml')            # under multiagent_est_con/config/
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('pose_topic_template', '/agent{}/pose')  # PoseStamped per agent
        self.declare_parameter('frame_id', 'world')                     # frame_id to stamp on output

        # Resolve config path
        pkg_share = get_package_share_directory('turtlesim_vicon')
        cfg_file = self.get_parameter('config_file').get_parameter_value().string_value
        cfg_path = os.path.join(pkg_share, 'config', cfg_file)
        if not os.path.exists(cfg_path):
            raise FileNotFoundError(f"Config file not found: {cfg_path}")

        # Load YAML
        with open(cfg_path, 'r') as f:
            data = yaml.safe_load(f)
        if 'network' not in data:
            raise KeyError("YAML must contain top-level key 'network'.")

        net = data['network']
        self.num_agents: int = int(net.get('num_agents', 0))
        if self.num_agents < 1:
            raise ValueError("network.num_agents must be >= 1")

        # Build adjacency (bi-directional)
        self.adj: Dict[int, Set[int]] = {i: set() for i in range(1, self.num_agents + 1)}
        for e in net.get('edges', []):
            i = int(e['i']); j = int(e['j'])
            if not (1 <= i <= self.num_agents and 1 <= j <= self.num_agents and i != j):
                self.get_logger().warn(f"Skipping invalid edge: {e}")
                continue
            self.adj[i].add(j)
            self.adj[j].add(i)

        # Pose storage: (x, y) from PoseStamped
        self.poses: Dict[int, Tuple[float, float]] = {}
    

        # Subscriptions
        pose_topic_template = self.get_parameter('pose_topic_template').get_parameter_value().string_value
        self.subs = []
        for i in range(1, self.num_agents + 1):
            topic = pose_topic_template.format(i)
            self.subs.append(
                self.create_subscription(PoseStamped, topic, lambda msg, i=i: self._pose_cb(i, msg), 10)
            )

        # Publishers (same topic names you used before)
        self.pubs: Dict[int, rclpy.publisher.Publisher] = {}
        for i in range(1, self.num_agents + 1):
            topic = f'agent{i}/neighbors'
            self.pubs[i] = self.create_publisher(RelativeNeighbors, topic, 10)

        # Timer
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.timer = self.create_timer(1.0 / rate_hz, self._on_timer)

        self.get_logger().info(
            f"Loaded {self.num_agents} agents from {cfg_path}; "
            f"subscribing to PoseStamped on '{pose_topic_template}'. "
            f"Publishing relative neighbors at {rate_hz:.1f} Hz."
        )

    def _pose_cb(self, agent_id: int, msg: PoseStamped):
        # Assume z = 0, extract x,y
        self.poses[agent_id] = (float(msg.pose.position.x), float(msg.pose.position.y))

    def _on_timer(self):
        if len(self.poses) < self.num_agents:
            return

        now = self.get_clock().now().to_msg()
        for i in range(1, self.num_agents + 1):
            if i not in self.poses:
                continue
            xi, yi = self.poses[i]

            rel_list: List[RelativeMeasurement] = []
            for j in sorted(self.adj.get(i, [])):
                if j not in self.poses:
                    continue
                xj, yj = self.poses[j]

                m = RelativeMeasurement()
                m.neighbor_id = j

                dx = xj - xi
                dy = yj - yi
                dij = math.sqrt(dx*dx + dy*dy)

                m.relative_position = Vector3(x=dx, y=dy, z=0.0)

                if dij > 1e-9:
                    m.relative_bearing = Vector3(x=dx/dij, y=dy/dij, z=0.0)
                else:
                    # i and j at same position (shouldn't happen, but safe)
                    m.relative_bearing = Vector3(x=math.inf, y=math.inf, z=math.inf)


                # m.relative_position = Vector3(x=xj - xi, y=yj - yi, z=0.0)
                # dij = np.linalg.norm(m.relative_position)
                # m.relative_bearing = Vector3(x=(xj - xi)/dij, y=(yj - yi)/dij, z=0.0)

                rel_list.append(m)

            out = RelativeNeighbors()
            out.header = Header(stamp=now, frame_id=self.frame_id)
            out.source_id = i
            out.neighbors = rel_list
            self.get_logger().info(f"values = {rel_list}")

            self.pubs[i].publish(out)


def main():
    rclpy.init()
    node = RelativeNeighborsPSNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
