#!/usr/bin/env python3

import os
import sys
import yaml
import argparse
import rclpy
import numpy as np
from rclpy.node import Node
from collections import defaultdict, deque
from geometry_msgs.msg import PoseStamped
from vicon_calibration_interfaces.srv import AssignTopic  # <-- Adjust to your actual package
from vicon_receiver.msg import Position

from pose_fusion.fusion import PoseFusion
from pose_fusion.spatial import Transformation
from rclpy.callback_groups import ReentrantCallbackGroup  # Import the callback group
from pose_fusion.conversion import quaternion_to_euler, matrix_to_quaternion
from datetime import datetime


class PoseCalibrationNode(Node):
    def __init__(self, calibration_file: str, sample_count: int):
        super().__init__('pose_calibration_node')
        self.sample_count = sample_count
        self.pose_buffers = defaultdict(lambda: deque(maxlen=sample_count))
        self.subscribers = {}

        if not os.path.isfile(calibration_file):
            self.get_logger().error(f"Calibration file not found: {calibration_file}")
            raise FileNotFoundError(calibration_file)

        self.calibration_file = calibration_file

        # Load the calibration data from the YAML file
        with open(calibration_file, 'r') as f:
            data = yaml.safe_load(f)
            self.robot_configs = data.get('robots', [])

        # Use a reentrant callback group for concurrent processing
        self.callback_group = ReentrantCallbackGroup()

        # Service
        self.srv = self.create_service(AssignTopic, 'calibrate_pose_bias', self._handle_calibration_request)
        self.get_logger().info("PoseCalibrationNode is ready and service is available.")

    def _handle_calibration_request(self, request: AssignTopic.Request, response: AssignTopic.Response):
        topic = request.topic
        samples = []
        collecting_samples = True  # Flag to track if we are collecting samples

        # Callback for processing messages from Vicon topics
        def callback(msg):
            samples.append(msg)
            self.get_logger().info(f'{len(samples)} samples collected')
            if len(samples) >= self.sample_count:
                # Once we have enough samples, stop collecting and destroy the subscriber
                self.get_logger().info(f"Collected {self.sample_count} samples, unsubscribing from {topic}")
                if topic in self.subscribers:
                    self.destroy_subscription(self.subscribers[topic])
                    del self.subscribers[topic]  # Remove from subscriber list
                nonlocal collecting_samples
                collecting_samples = False  # Stop collecting samples

        # Create the subscriber dynamically inside the service callback
        if topic not in self.subscribers:
            self.subscribers[topic] = self.create_subscription(
                Position,
                topic,
                callback,
                10,
                callback_group=self.callback_group  # Associate the subscriber with the callback group
            )
            self.get_logger().info(f"Subscribed to Vicon topic: {topic}")

        # Wait until enough samples are collected
        while collecting_samples:
            self.get_logger().info(f"Waiting for samples: Collected {len(samples)} / {self.sample_count}")
            rclpy.spin_once(self)  # Process incoming messages
        
        # Once enough samples are collected, proceed with fusion
        transforms = []
        covariances = []
        for pose in samples:
            self.get_logger().info(f'{pose}')
            T = Transformation(
                rotation=[pose.w, pose.x_rot, pose.y_rot, pose.z_rot],
                translation=[pose.x_trans, pose.y_trans, pose.z_trans]
            )
            transforms.append(T.matrix())
            covariances.append(0.01 * np.eye(6))  # Assume small uncertainty

        fusion = PoseFusion(
            transform_dataset=transforms,
            covariance_dataset=covariances
        )
        fused_pose, _ = fusion.fuse_poses()

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'vicon'

        q = matrix_to_quaternion(fused_pose[0:3, 0:3])
        msg.pose.orientation.w = q[0]
        msg.pose.orientation.x = q[1]
        msg.pose.orientation.y = q[2]
        msg.pose.orientation.z = q[3]
            
        msg.pose.position.x = fused_pose[0, 3]
        msg.pose.position.y = fused_pose[1, 3]
        msg.pose.position.z = fused_pose[2, 3]
        
        response.pose = msg
        self.get_logger().info(f"Calibration complete for topic: {topic}")

        # Save the updated calibration data to the YAML file
        self.update_calibration_file(topic, fused_pose)

        return response

    def update_calibration_file(self, topic: str, fused_pose: np.ndarray):
        """Update the calibration YAML file with the new calibration data."""
        with open(self.calibration_file, 'r') as f:
            data = yaml.safe_load(f)
        # Find the robot corresponding to the topic
        robot = None
        for robot_entry in data['robots']:
            a = robot_entry['vicon_topic']
            self.get_logger().info(f'{a}')
            self.get_logger().info(f'{topic}')
            
            if robot_entry['vicon_topic'] == topic:
                
        
                robot = robot_entry
                break

        if robot:
            # Update the robot's calibration data with the new pose
            robot['position']['x'] = float(fused_pose[0, 3])
            robot['position']['y'] = float(fused_pose[1, 3])
            robot['position']['z'] = float(fused_pose[2, 3])
            q = matrix_to_quaternion(fused_pose[0:3, 0:3])
            robot['orientation']['w'] = float(q[0])
            robot['orientation']['x'] = float(q[1])
            robot['orientation']['y'] = float(q[2])
            robot['orientation']['z'] = float(q[3])
            
            
            # Add the current timestamp
            robot['timestamp'] = datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%SZ')

            # Save the updated data back to the YAML file
            with open(self.calibration_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            self.get_logger().info(f"Calibration data for {topic} updated in {self.calibration_file}")
        else:
            self.get_logger().warn(f"No robot found for topic: {topic}, unable to update YAML file.")

def parse_cli_args():
    from rclpy.utilities import remove_ros_args
    parser = argparse.ArgumentParser(description="Pose Calibration Node")
    parser.add_argument('-f', '--file', type=str, required=True,
                        help='Path to calibration_info.yaml')
    parser.add_argument('--sample_count', type=int, default=30,
                        help='Number of pose samples to collect')
    clean_args = remove_ros_args(sys.argv)[1:]  # remove ROS args before parsing
    return parser.parse_args(clean_args)

def main(args=None):
    cli_args = parse_cli_args()
    rclpy.init(args=args)

    try:
        node = PoseCalibrationNode(
            calibration_file=cli_args.file,
            sample_count=cli_args.sample_count
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
