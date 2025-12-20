#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
from math import cos, sin
import yaml
from pose_fusion.circle_estimate import estimate_3d_circle_center
from scipy.spatial.transform import Rotation

data = {
    'frame_id': ' ',
    'orientation': {
        'w': 0.0,
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
    },
    'position': { 
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
    },
    'timestamp': 0.0
}

class ViconEmulatorSpin(Node):
    def __init__(self):
        super().__init__('vicon_emulator_spin')

        # Parameters
        self.freq = 50.0  # Hz
        self.offset_x = 0.2  # meters (marker offset from center)
        self.offset_y = 0.1
        self.offset_delta = np.pi/2
        self.angular_velocity = 0.3 # rad/s
        self.marker_x = 0.0
        self.marker_y = 0.0
        self.points = []
        self.rotations = []

        self.publisher_ = self.create_publisher(PoseStamped, '/vicon/limo/pose', 10)
        self.timer_ = self.create_timer(1.0 / self.freq, self.publish_offset_pose)
        self.t = 0.0
        self.file = "config.yaml"
        with open(self.file, 'w') as file:
            return

        self.get_logger().info('Vicon emulator started with offset (%.2f, %.2f)' % (self.offset_x, self.offset_y))

    def publish_offset_pose(self):
        theta = (self.angular_velocity * self.t) # current heading of robot
        if theta <= 2 * np.pi:
            R = np.array([
                [cos(theta), -sin(theta)],
                [sin(theta),  cos(theta)]
            ])

            R_offset = np.array([
                [cos(self.offset_delta), -sin(self.offset_delta)],
                [sin(self.offset_delta),  cos(self.offset_delta)]
            ])

            offset_velocity = np.array(np.cross([0, 0, self.angular_velocity], [self.offset_x, self.offset_y, 0]))
            rotated_offset_velocity = R @ R_offset @ offset_velocity[0:2]

            # Marker position relative to world (robot is rotating at origin)
            marker_x_velocity = rotated_offset_velocity[0]
            marker_y_velocity = rotated_offset_velocity[1]

            self.marker_x += marker_x_velocity * (1.0 / self.freq)
            self.marker_y += marker_y_velocity * (1.0 / self.freq)

            # Orientation as quaternion (yaw only)
            qz = sin(theta / 2.0)
            qw = cos(theta / 2.0)

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            msg.pose.position.x = self.marker_x
            msg.pose.position.y = self.marker_y
            msg.pose.position.z = 0.0

            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw

            data['frame_id'] = msg.header.frame_id
            data['position']['x'] = float(msg.pose.position.x)
            data['position']['y'] = float(msg.pose.position.y)
            data['position']['z'] = 0.0
            data['orientation']['w'] = msg.pose.orientation.w 
            data['orientation']['x'] = msg.pose.orientation.x 
            data['orientation']['y'] = msg.pose.orientation.y
            data['orientation']['z'] = msg.pose.orientation.z
            data['timestamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
            with open(self.file, 'a') as file:
                yaml.dump(data, file, default_flow_style=False)
            self.publisher_.publish(msg)
            self.t += 1.0 / self.freq
            self.points.append((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
            self.rotations.append((msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
        else:
            circle = estimate_3d_circle_center(np.array(self.points))
            compensation = np.array([0.0, 0.0, 0.0])
            n = 0
            for point, rotation in zip(np.array(self.points), np.array(self.rotations)):
                r = Rotation.from_quat(rotation)
                compensation += r.as_matrix().T @ (circle - point) 
                n += 1
            compensation /= n
            with open(self.file, 'w') as file:
                yaml.dump(float(compensation[0]), file, default_flow_style=False)
                yaml.dump(float(compensation[1]), file, default_flow_style=False)
                yaml.dump(float(compensation[2]), file, default_flow_style=False)
            
            return


def main(args=None):
    rclpy.init(args=args)
    node = ViconEmulatorSpin()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
