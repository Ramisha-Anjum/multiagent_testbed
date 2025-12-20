#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vicon_calibration_interfaces.srv import AssignTopic
import numpy as np
from vicon_receiver.msg import Position
from rclpy.callback_groups import ReentrantCallbackGroup
from pose_fusion import circle_estimate

class SpinCalibrationServer(Node):
    def __init__(self):
        super().__init__('spin_calibration_server')
        self.callback_group = ReentrantCallbackGroup()
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10, callback_group=self.callback_group)
        self.service = self.create_service(AssignTopic, 'spin_calibration', self.handle_spin_request)
        self.pose_buffer = []
        self.subscription = None
        self.twist = Twist()
        self.theta = 0.0
        self.target_rotation = 2 * np.pi # one full revolution
        self.omega = 0.3
        self.dt = 0.1 # time step in seconds
        self.spin = False

        # Create a timer
        self.timer = self.create_timer(self.dt, self.spin_step, callback_group=self.callback_group)

        self.get_logger().info("Spin Calibration Service ready.")

    def spin_step(self):
        if self.spin:
            remaining = self.target_rotation - self.theta
            if remaining <= 0:
                # Stop spinning
                self.get_logger().info("Full rotation completed!")
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
                self.spin = False
                # Output circle trajectory points
                estimated_center = circle_estimate.estimate_3d_circle_center(np.array(self.pose_buffer))
                # test_circle_estimate.visualize(np.array(self.pose_buffer), estimated_center)
                self.get_logger().info(estimated_center)
            else:
                # Minimize angular velocity to prevent overshoot/rotating more than one revolution
                max_omega = min(self.omega, remaining / self.dt)
                self.twist.angular.z = max_omega
                self.cmd_vel_pub.publish(self.twist)
                self.theta += max_omega * self.dt
                self.get_logger().info(f"Theta: {self.theta:.2f} rad")

    def handle_spin_request(self, request, response):
        self.theta = 0.0
        self.pose_buffer = []
        self.get_logger().info(f"Starting spin using VICON topic: {request.topic}")

        # Set up pose subscriber
        if self.subscription:
            self.destroy_subscription(self.subscription)
        self.subscription = self.create_subscription(Position, request.topic, self.pose_callback, 10, callback_group=self.callback_group)

        # Spin in place
        self.spin = True

        return response

    def pose_callback(self, msg):
        self.pose_buffer.append([msg.x_trans, msg.y_trans, msg.z_trans])

def main(args=None):
    rclpy.init(args=args)
    node = SpinCalibrationServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()