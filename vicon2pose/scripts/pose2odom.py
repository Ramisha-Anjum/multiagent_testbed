#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class PoseToOdom(Node):
    def __init__(self):
        super().__init__('pose2odom')

        self.declare_parameter('pose_topic', '/pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('child_frame_id', 'base_link')

        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

        self.publisher = self.create_publisher(Odometry, odom_topic, 10)
        self.subscription = self.create_subscription(PoseStamped, pose_topic, self.pose_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)

    def pose_callback(self, msg: PoseStamped):
        # Odometry message
        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose = msg.pose
        self.publisher.publish(odom)

        # TF message
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
