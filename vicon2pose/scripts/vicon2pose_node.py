#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from vicon_receiver.msg import Position
from geometry_msgs.msg import PoseStamped
import argparse
import sys, os
import yaml
import numpy as np
from pose_fusion.conversion import quaternion_to_euler, matrix_to_quaternion
from datetime import datetime

# Clean topic name to use as node name
def clean_path(text):
    if text.startswith('/'):
        text = text[1:]
    return text.replace('/', '_')

def quaternion_rotate_vector(q, v):
    """
    Rotate vector v by quaternion q.
    q: [w, x, y, z]
    v: [x, y, z]
    Returns rotated vector in world frame.
    """
    w, x, y, z = q
    q_vec = np.array([x, y, z])
    v = np.array(v)
    
    uv = np.cross(q_vec, v)
    uuv = np.cross(q_vec, uv)
    
    return v + 2 * (w * uv + uuv)

class ViconToPose(Node):

    def __init__(self, vicon_topic, frame_id, calibration_file):
        super().__init__(clean_path(vicon_topic)+'_vicon2pose')

        self.vicon_topic = vicon_topic
        self.frame_id = frame_id
        self.calibration_file = calibration_file

        # Load calibration data
        self.calibration_data = self.load_calibration_data()

        self.subscription = self.create_subscription(
            Position,
            self.vicon_topic,
            self.listener_callback,
            10)

        # self.publisher = self.create_publisher(
        #     PoseStamped,
        #     self.vicon_topic[-6:] + '/pose',
        #     10)

        # self.get_logger().info(f'Subscribed to {self.vicon_topic}, publishing PoseStamped to {self.vicon_topic + "/pose"}')

        # Derive a name from the last part of the Vicon topic
        base_name = self.vicon_topic.rstrip('/').split('/')[-1]

        pose_topic = f'/{base_name}/pose'

        self.publisher = self.create_publisher(
            PoseStamped,
            pose_topic,
            10
        )

        self.get_logger().info(
            f'Subscribed to {self.vicon_topic}, publishing PoseStamped to {pose_topic}'
        )



    def load_calibration_data(self):
        if os.path.isfile(self.calibration_file):
            with open(self.calibration_file, 'r') as file:
                data = yaml.safe_load(file)
                calibration_data = {}
                for robot in data['robots']:
                    calibration_data[robot['vicon_topic']] = robot
            return calibration_data
        else:
            self.get_logger().warn(f"Calibration file not found: {self.calibration_file}")
            return {}

    def listener_callback(self, msg: Position):
        # Skip if quaternion is all zeros
        if msg.w == 0.0 and msg.x_rot == 0.0 and msg.y_rot == 0.0 and msg.z_rot == 0.0:
            self.get_logger().warn(f"Ignored invalid quaternion from topic {self.vicon_topic}")
            return

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.frame_id

        # Raw Vicon data
        position = {'x': msg.x_trans, 'y': msg.y_trans, 'z': msg.z_trans}
        orientation = {'x': msg.x_rot, 'y': msg.y_rot, 'z': msg.z_rot, 'w': msg.w}

        if self.vicon_topic in self.calibration_data.keys():
            calib = self.calibration_data[self.vicon_topic]

            if calib.get('orientation') is not None:
                q_bias = [
                    calib['orientation']['w'],
                    calib['orientation']['x'],
                    calib['orientation']['y'],
                    calib['orientation']['z']
                ]

                # Rotate position offset from body frame to world frame
                dp = [
                    calib['position']['x'],
                    calib['position']['y'],
                    calib['position']['z']
                ]
                dp_world = quaternion_rotate_vector(q_bias, dp)

                # Compensated position
                position['x'] -= dp_world[0]
                position['y'] -= dp_world[1]
                position['z'] -= dp_world[2]

                # Compensated orientation: q_corrected = q_bias * q_vicon
                q_vicon = [orientation['w'], orientation['x'], orientation['y'], orientation['z']]
                q_compensated = self.quaternion_multiply(q_bias, q_vicon)

                orientation['w'] = q_compensated[0]
                orientation['x'] = q_compensated[1]
                orientation['y'] = q_compensated[2]
                orientation['z'] = q_compensated[3]

        # Convert mm to meters
        pose_msg.pose.position.x = position['x'] / 100
        pose_msg.pose.position.y = position['y'] / 100
        pose_msg.pose.position.z = position['z'] / 100

        pose_msg.pose.orientation.x = orientation['x']
        pose_msg.pose.orientation.y = orientation['y']
        pose_msg.pose.orientation.z = orientation['z']
        pose_msg.pose.orientation.w = orientation['w']

        self.publisher.publish(pose_msg)

    def quaternion_multiply(self, q1, q2):
        """
        Multiply two quaternions: q1 * q2
        Both q1 and q2 are in [w, x, y, z] format
        """
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        return [w, x, y, z]

def parse_cli_args():
    from rclpy.utilities import remove_ros_args
    parser = argparse.ArgumentParser(description="Vicon to Pose Node")
    parser.add_argument('--vicon_topic', type=str, required=True)
    parser.add_argument('--frame_id', type=str, default='vicon')
    parser.add_argument('--calibration_file', type=str, required=True,
                        help='Path to the YAML calibration file')

    parsed_args = remove_ros_args(args=sys.argv)
    cli_args = parser.parse_args(parsed_args[1:])
    return cli_args

def main(args=None):
    cli_args = parse_cli_args()
    rclpy.init(args=args)
    node = ViconToPose(cli_args.vicon_topic, cli_args.frame_id, cli_args.calibration_file)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
