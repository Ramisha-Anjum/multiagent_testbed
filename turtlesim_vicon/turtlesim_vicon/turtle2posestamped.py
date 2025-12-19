#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Node: turtlesim_pose_bridge
# Package: multiagent_est_con
#
# Subscribes: /turtle{i}/pose (turtlesim/Pose)
# Publishes : <output_topic_template>.format(i) (geometry_msgs/PoseStamped)
# Parameters:
#   - config_file            : config.yaml under this package's config/
#   - output_topic_template  : default '/agent{}/pose'
#   - frame_id               : default 'world'

import os
import math
import yaml
from typing import Dict

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from turtlesim.msg import Pose as TurtlePose


class TurtlesimPoseBridge(Node):
    def __init__(self):
        super().__init__('turtlesim_pose_bridge')

        # Parameters
        self.declare_parameter('config_file', 'config.yaml')
        self.declare_parameter('output_topic_template', '/agent{}/pose')
        self.declare_parameter('frame_id', 'world')

        pkg_share = get_package_share_directory('multiagent_est_con')
        cfg_file = self.get_parameter('config_file').get_parameter_value().string_value
        cfg_path = os.path.join(pkg_share, 'config', cfg_file)
        if not os.path.exists(cfg_path):
            raise FileNotFoundError(f"Config file not found: {cfg_path}")

        with open(cfg_path, 'r') as f:
            data = yaml.safe_load(f)
        net = data['network']
        self.num_agents: int = int(net.get('num_agents', 0))
        if self.num_agents < 1:
            raise ValueError("network.num_agents must be >= 1")

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        out_tpl = self.get_parameter('output_topic_template').get_parameter_value().string_value

        # Publishers for PoseStamped
        self.pubs: Dict[int, rclpy.publisher.Publisher] = {}
        for i in range(1, self.num_agents + 1):
            topic = out_tpl.format(i)
            self.pubs[i] = self.create_publisher(PoseStamped, topic, 10)

        # Subscriptions to turtlesim Pose
        self.subs = []
        for i in range(1, self.num_agents + 1):
            topic = f'/turtle{i}/pose'
            self.subs.append(
                self.create_subscription(TurtlePose, topic, lambda msg, i=i: self._pose_cb(i, msg), 10)
            )

        self.get_logger().info(
            f"Bridging turtlesim/Pose -> PoseStamped for {self.num_agents} turtles. "
            f"Output template: '{out_tpl}', frame_id: '{self.frame_id}'."
        )

    @staticmethod
    def _yaw_to_quat(theta: float) -> Quaternion:
        half = 0.5 * theta
        return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))

    def _pose_cb(self, agent_id: int, msg: TurtlePose):
        # Build PoseStamped (z=0)
        ps = PoseStamped()
        ps.header = Header()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.frame_id

        ps.pose = Pose()
        ps.pose.position = Point(x=float(msg.x), y=float(msg.y), z=0.0)
        ps.pose.orientation = self._yaw_to_quat(float(msg.theta))

        self.pubs[agent_id].publish(ps)


def main():
    rclpy.init()
    node = TurtlesimPoseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
