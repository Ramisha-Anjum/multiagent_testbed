#!/usr/bin/env python3

import math
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import cv2
from cv_bridge import CvBridge


class WebcamArucoBearing(Node):
    """
    Simple ROS2 node to:
      - Subscribe to a webcam image topic
      - Detect an ArUco marker with a given ID
      - Compute horizontal bearing sigma_rel in camera frame
      - Compute b = [cos(sigma_rel), sin(sigma_rel)] assuming yaw = 0
      - Publish a Marker arrow in 'map' frame representing b
    """

    def __init__(self):
        super().__init__("webcam_aruco_bearing")

        # Parameters
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("marker_id", 2)
        self.declare_parameter("hfov_deg", 60.0)  # approximate horizontal FOV if no intrinsics

        self.image_topic: str = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.marker_id: int = (
            self.get_parameter("marker_id").get_parameter_value().integer_value
        )
        self.hfov_deg: float = (
            self.get_parameter("hfov_deg").get_parameter_value().double_value
        )

        self.get_logger().info(
            f"Using image_topic={self.image_topic}, marker_id={self.marker_id}, "
            f"hfov_deg={self.hfov_deg}"
        )

        # CvBridge
        self.bridge = CvBridge()

        # State: last bearing and b-vector
        self.sigma_rel: Optional[float] = None
        self.b_vec: Optional[np.ndarray] = None

        # Publisher for RViz marker
        self.marker_pub = self.create_publisher(Marker, "webcam_bearing_marker", 10)

        # Subscriber to webcam image
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )

        self.get_logger().info("webcam_aruco_bearing node started.")

        self.last_log_time = self.get_clock().now().nanoseconds * 1e-9


    # ----------------- Main image callback -----------------

    def image_callback(self, msg: Image):

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_log_time > 1.0:
            self.get_logger().info("Receiving frames...")
            self.last_log_time = now

        """Process incoming webcam image, detect ArUco, compute bearing + marker."""
        # Convert ROS image -> OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Failed to convert image: {e}")
            return

        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # ArUco detection
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)


        # # --- ArUco detection (new-style API) ---
        # aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        # aruco_params = cv2.aruco.DetectorParameters()
        # detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        # corners, ids, _ = detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            self.get_logger().info("No ArUco markers detected in webcam image")
            return

        # if ids is None:
        #     self.get_logger().info("No ArUco markers detected in webcam image")
        #     return

        ids_flat = ids.flatten()

        # Look for the desired marker_id
        for marker_corners, marker_id in zip(corners, ids_flat):
            if marker_id == self.marker_id:
                # Compute pixel center of the marker
                pts = marker_corners.reshape(-1, 2)
                u = float(np.mean(pts[:, 0]))  # pixel x
                v = float(np.mean(pts[:, 1]))  # pixel y (unused here)

                h, w = gray.shape[:2]
                cx = w / 2.0

                # Approximate focal length fx using HFOV if we have no camera_info
                # fx = w / (2 * tan(HFOV/2))
                hfov_rad = math.radians(self.hfov_deg)
                fx = w / (2.0 * math.tan(hfov_rad / 2.0))

                # Horizontal bearing: alpha = atan2(u - cx, fx)
                alpha = math.atan2(u - cx, fx)

                self.sigma_rel = alpha
                self.b_vec = np.array(
                    [math.cos(alpha), math.sin(alpha)],
                    dtype=float,
                )

                self.get_logger().info(
                    f"Detected marker id={marker_id}: u={u:.1f}, cx={cx:.1f}, "
                    f"alpha={alpha:.3f} rad, b={self.b_vec}"
                )

                # Publish marker arrow for RViz
                self.publish_marker()
                break

    # ----------------- Marker publishing -----------------

    def publish_marker(self):
        """Publish an arrow in 'map' frame representing b_vec from origin."""
        if self.b_vec is None:
            return

        marker = Marker()
        marker.header.frame_id = "map"  # Set RViz Fixed Frame = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "webcam_bearing"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Start at origin
        marker.points.append(Point(x=0.0, y=0.0, z=0.0))

        # End at scaled b_vec
        scale_len = 1.0
        end_x = float(scale_len * self.b_vec[0])
        end_y = float(scale_len * self.b_vec[1])
        marker.points.append(Point(x=end_x, y=end_y, z=0.0))

        # Shaft/head scales
        marker.scale.x = 0.05  # shaft diameter
        marker.scale.y = 0.1   # head diameter
        marker.scale.z = 0.1   # head length

        # Red arrow
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = WebcamArucoBearing()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
