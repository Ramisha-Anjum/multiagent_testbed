#!/usr/bin/python3

# other libraries
import numpy as np

# package module
from pose_fusion.spatial import Transformation
from pose_fusion.utils import PoseDataSynthesizer 
from pose_fusion.fusion import PoseFusion

# RCLPY libraries, classes, functions
import rclpy
from rclpy.node import Node


class OfflineFusion(Node):
    """
    ROS2 node that generates a synthetic dataset of poses and performs pose fusion.

    The node generates a synthetic dataset of poses using a transformation matrix and a covariance matrix,
    and then performs pose fusion on the dataset. The fused pose is then compared with the ground truth pose.

    Parameters:
        None

    Attributes:
        None
    """
    def __init__(self):
        """
        Initializes the OfflineFusion node.

        Generates a synthetic dataset of poses using a transformation matrix and a covariance matrix,
        and then performs pose fusion on the dataset. The fused pose is then compared with the ground truth pose.

        Args:
            None
        """
        super().__init__('offline_fusion')

        # Define a rotation matrix
        rpy = [0,np.pi/2,np.pi/2]
        position = [1,2,3]
        
        T = Transformation(rotation=rpy,translation=position) 
        cov = np.diag([0.1,0.1,0.1,0.05,0.05,0.05])

        data_sythesizer = PoseDataSynthesizer()
        num_meas_poses = 1000
        
        transforms,covariance_matrices = data_sythesizer.synthesize(nominal_pose=T.matrix(),covariance=cov,num=num_meas_poses)
        fusion = PoseFusion(
            transform_dataset=transforms,
            covariance_dataset=covariance_matrices
        )
        fused_pose,cov = fusion.fuse_poses(method="avg_tan_vec")

        self.get_logger().info(f"Estimated Transform:     \n{fused_pose}")
        self.get_logger().info(f"Ground Truth Transform:  \n{fused_pose}")

# Define the main function that will run when this script is execute
def main(args=None):
    rclpy.init(args=args)
    node = OfflineFusion()
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
