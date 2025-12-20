#!/usr/bin/python3

# other libraries
import numpy as np

# package module
from pose_fusion.fusion import PoseFusion
from pose_fusion.spatial import Transformation
from pose_fusion.conversion import quaternion_to_euler,matrix_to_quaternion

# RCLPY libraries, classes, functions
import rclpy
from rclpy.node import Node

# ROS Package
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry 
from tf2_ros.transform_broadcaster import TransformBroadcaster

class PoseFusionNode(Node):
    """
    ROS2 Node to fuse multiple pose data streams.
    """
    def __init__(self):
        """
        Initialize ROS2 Node and set up subscriptions and publishers.
        """
        super().__init__('pose_fusion')
        self.declare_parameter('fusion_method',value='man_opt')
        
        self.sub = self.create_subscription(TFMessage,'tf',self.fuse_pose,10)
        self.broadcaster = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry,'fused_odom',10)
        
    def fuse_pose(self,msg:TFMessage):
        """
        Fuse multiple pose data streams into a single pose, and publish to topic 'fused_odom'
        """
        # initialize lists to store poses and covariances for each pose stream
        transforms = []
        covariance_matrices = []
        # iterate over transforms in TFMessage
        for transform in msg.transforms:
            # check if child frame id starts with 'pose', indicating a pose data stream
            if str(transform.child_frame_id).startswith('pose'):
                # perform conversion on the transformations
                R,P,Y = quaternion_to_euler([
                    transform.transform.rotation.w,
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z
                ])
                translation = [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ]
                T = Transformation(rotation=[R,P,Y],translation=translation) 
                cov = np.diag([0.1,0.1,0.1,0.05,0.05,0.05])
                transforms.append(T.matrix())
                covariance_matrices.append(cov)
        # check if any valid poses were found
        if len(transforms)>0:
            # create PoseFusion object with transformation and covariance data
            fusion = PoseFusion(
                transform_dataset=transforms,
                covariance_dataset=covariance_matrices
            )
            # fuse poses and get resulting pose and covariance matrix
            fused_pose,fused_cov = fusion.fuse_poses(method=self.get_parameter('fusion_method').value)
            # put the results into corresponding topics
            fused_transform = TransformStamped()
            odom = Odometry()
            # write to Header
            header =Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'world'
            fused_transform.header = header
            odom.header = header
            # write to covariance
            cov_shape = fused_cov.shape
            # must be a 1-d array
            odom.pose.covariance = np.reshape(fused_cov,(cov_shape[0]*cov_shape[1]))
            # write to child_frame_id
            child_frame_id = 'fused_odom'
            odom.child_frame_id = child_frame_id
            fused_transform.child_frame_id = child_frame_id
            # write to the transform
            q = matrix_to_quaternion(fused_pose[0:3,0:3])
            fused_transform.transform.rotation.w = q[0]
            fused_transform.transform.rotation.x = q[1]
            fused_transform.transform.rotation.y = q[2]
            fused_transform.transform.rotation.z = q[3]
            fused_transform.transform.translation.x = fused_pose[0,3]
            fused_transform.transform.translation.y = fused_pose[1,3]
            fused_transform.transform.translation.z = fused_pose[2,3]
            # write to odom's pose
            odom.pose.pose.orientation.w = q[0]
            odom.pose.pose.orientation.x = q[1]
            odom.pose.pose.orientation.y = q[2]
            odom.pose.pose.orientation.z = q[3]
            odom.pose.pose.position.x = fused_pose[0,3]
            odom.pose.pose.position.y = fused_pose[1,3]
            odom.pose.pose.position.z = fused_pose[2,3]
            # broadcast the fused transform and publish the odom
            self.broadcaster.sendTransform(fused_transform)
            self.pub.publish(odom)

# Define the main function that will run when this script is execute
def main(args=None):
    rclpy.init(args=args)
    node = PoseFusionNode()
    try:
        while rclpy.ok(): # while the node isn't shut down
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node has stopped cleanly.')
    except SystemExit:
        node.get_logger().info('Node is complete.')
    except BaseException as exc:
        type = exc.__class__.__name__
        node.get_logger().error(f'{type} exception in node has occured.')
        raise # raise without argument = raise the last exception
    finally:
        node.destroy_node()
        rclpy.shutdown() 
        
# If this script is being run as the main program, call the main function
if __name__=='__main__':
    main()
