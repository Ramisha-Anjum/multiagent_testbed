#!/usr/bin/env python3

import os
import math
import yaml
from typing import Dict, Tuple, Set
import numpy as np
from scipy.integrate import solve_ivp


import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, Imu
from visualization_msgs.msg import Marker


import cv2
from cv_bridge import CvBridge


# # Gains (tune later)
# kp = 0.8
# kw = 1.5
# ki = 0.2
# rho = 1.0

class CameraBearing(Node):
    def __init__(self):
        super().__init__("camera_bearing")

        # ---------------- Params & config ----------------
        self.declare_parameter("config_file", "config.yaml")

        pkg_share = get_package_share_directory("image_detection")
        cfg_file = self.get_parameter("config_file").get_parameter_value().string_value
        cfg_path = os.path.join(pkg_share, "config", cfg_file)
        if not os.path.exists(cfg_path):
            raise FileNotFoundError(f"Config file not found: {cfg_path}")

        with open(cfg_path, "r") as f:
            data = yaml.safe_load(f)

        net = data["network"]
        self.num_agents: int = int(net.get("num_agents", 0))
        if self.num_agents < 1:
            raise ValueError("network.num_agents must be >= 1")

      
        # Build adjacency (one-directional)
        self.adj: Dict[int, Set[int]] = {i: set() for i in range(1, self.num_agents + 1)}

        for e in net.get('edges', []):
            i = int(e['i']); j = int(e['j'])

            # Basic validation
            if not (1 <= i <= self.num_agents and 1 <= j <= self.num_agents and i != j):
                self.get_logger().warn(f"Skipping invalid edge (agent ids): {e}")
                continue

            # directed adjacency
            self.adj[i].add(j)

        
        # ---------------- State storage ----------------
        # World positions and yaws for each robot
        self.positions: Dict[int, np.ndarray] = {}
        self.yaws: Dict[int, float] = {}

        # Initialize positions and yaw from config so we have something even      
        # if /robotX/pose is not publishing yet.
        init_positions = net.get("initial_position", [])
        for i, p in enumerate(init_positions, start=1):
            x = float(p["x"])
            y = float(p["y"])
            self.positions[i] = np.array([x, y], dtype=float)
            self.yaws[i] = 0.0  # assume facing along +x initially
        self.get_logger().info(
            f"Initialized from config: positions={self.positions}, yaws={self.yaws}"
        )
        
        # Camera intrinsics per robot i: K_i (3x3)
        self.camera_matrices: Dict[int, np.ndarray] = {}

        # Relative bearing in robot 1 frame:
        # sigma_rel[(1, 2)] = bearing angle from robot 1 to robot 2 in robot1's camera frame
        self.sigma_rel: Dict[Tuple[int, int], float] = {}

        # Final world-frame unit vectors from i to j:
        # b_ij[(i, j)] = np.array([cos(sigma_ij_world), sin(sigma_ij_world)])
        self.b_ij: Dict[Tuple[int, int], np.ndarray] = {}

        # # beta_ij[(i, j)] = b_ij expressed in robot i body frame (optional)
        # #self.beta_ij: Dict[Tuple[int, int], np.ndarray] = {}
        # self.betai: Dict[int, np.ndarray] = {i: np.zeros(2, dtype=float) for i in range(1, self.num_agents + 1)}


        self.bridge = CvBridge()


        # # Publisher        
        # self.marker_pub = self.create_publisher(Marker, "b12_marker", 10)

        # # Publishers
        # self.pubs: Dict[int, rclpy.publisher.Publisher] = {}
        # for i in range(1, self.num_agents + 1):
        #     topic = f'/robot{i}/cmd_vel'
        #     self.pubs[i] = self.create_publisher(Twist, topic, 10)

        # ---------------- Pose subscriptions ----------------
        # For real Turtlebot4, you might map from /odom or /pose:
        #   remap /robot1/pose -> /tb4_1/pose, etc., in your launch file.
        # self.pose_subs = []
        # for i in range(1, self.num_agents + 1):
        #     topic = f"/robot{i}/pose"
        #     self.get_logger().info(f"Subscribing pose of robot {i} from {topic}")
        #     self.pose_subs.append(
        #         self.create_subscription(
        #             PoseStamped,
        #             topic,
        #             lambda msg, i=i: self.pose_callback(i, msg),
        #             10,
        #         )
        #     )
        self.odom_subs = []
        for i in range(1, self.num_agents + 1):
            odom_topic = f"/robot{i}/odom"
            self.get_logger().info(f"Subscribing orientation of robot {i} from {odom_topic}")
            self.odom_subs.append(
                self.create_subscription(
                    Odometry,
                    odom_topic,
                    lambda msg, i=i: self.odom_callback(i, msg),
                    10,
                )
            )

        # ---------------- Camera subscriptions for each robot----------------
        # Adapt these topic names to your TurtleBot4:
        #   e.g. "/camera/image_raw" and "/camera/camera_info" or
        #        "/oakd/rgb/image_raw", etc.
        self.camera_info_subs = []
        for i in range(1, self.num_agents + 1):
            cam_info_topic = f"/robot{i}/oakd/rgb/preview/camera_info"
            #cam_info_topic = f"/robot{i}/camera/camera_info"
            self.get_logger().info(f"Subscribing camera info of robot {i} from {cam_info_topic}")
            self.camera_info_subs.append(
                self.create_subscription(
                    CameraInfo,
                    cam_info_topic,
                    lambda msg, i=i: self.camera_info_callback(i, msg),
                    10,
                )
            )
        self.image_subs = []
        for i in range(1, self.num_agents + 1):
            img_topic = f"/robot{i}/oakd/rgb/preview/image_raw"
            #img_topic = f"/robot{i}/camera/image_raw"
            self.get_logger().info(f"Subscribing camera image of robot {i} from {img_topic}")
            self.image_subs.append(
                self.create_subscription(
                    Image,
                    img_topic,
                    lambda msg, i=i: self.image_callback(i, msg),
                    10,
                )
            )

        # ---------------- Timer ----------------
        self.period = 0.05  # 20 Hz
        self.timer = self.create_timer(self.period, self.timer_callback)

        self.get_logger().info("collisionfree_algorithm node started.")


    # --------- Helpers ---------

    # @staticmethod
    # def quaternion_to_yaw(q: Quaternion) -> float:
    #     """Convert quaternion to yaw (rotation around z)."""
    #     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    #     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    #     return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def quaternion_to_yaw(w,x,y,z) -> float:
        """Convert quaternion to yaw (rotation around z)."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    # --------- Callbacks ---------

    # def pose_callback(self, agent_id: int, msg: PoseStamped):
    #     """Store world position of each robot i."""
    #     pos = msg.pose.position
    #     #q = msg.pose.orientation

    #     self.positions[agent_id] = np.array([float(pos.x), float(pos.y)], dtype=float)
    #     #self.yaws[agent_id] = self.quaternion_to_yaw(q)

    # def imu_callback(self, agent_id: int, msg: Imu):
    #     """Store world yaw of each robot i."""
    #     w = msg.orientation.w
    #     x = msg.orientation.x
    #     y = msg.orientation.y
    #     z = msg.orientation.z
    #     self.yaws[agent_id] = self.quaternion_to_yaw(w,x,y,z)
    #     #self.yaws[agent_id] = self.quaternion_to_yaw(q)

    def odom_callback(self, agent_id: int, msg: Odometry):
        """Store world yaw of each robot i."""
        w = msg.pose.pose.orientation.w
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        self.yaws[agent_id] = self.quaternion_to_yaw(w,x,y,z)
        #self.yaws[agent_id] = self.quaternion_to_yaw(q)


    def camera_info_callback(self, agent_id: int, msg: CameraInfo):
        """Save the camera intrinsic matrix K for robot i."""
        if agent_id not in self.camera_matrices:
            K = np.array(msg.k, dtype=float).reshape(3, 3)
            self.camera_matrices[agent_id] = K
            self.get_logger().info(f"Camera intrinsics set for robot {agent_id}:\n{K}")
    
    def image_callback(self, agent_id: int, msg: Image):
        """
        Process camera image from robot i, detect other robots' ArUco markers,
        and compute bearing angles sigma_ij in robot i frame.

        Assumption: robot j carries ArUco marker with ID = j.
        """
        if agent_id not in self.camera_matrices:
            # Wait until we have camera info for this robot
            return

        # Convert ROS2 image -> OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(
                f"Failed to convert image for robot {agent_id}: {e}"
            )
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # --- ArUco detection (new-style API) ---
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is None:
            # No markers detected in this frame
            return

        ids_flat = ids.flatten()
        K = self.camera_matrices[agent_id]
        fx = K[0, 0]
        cx = K[0, 2]

        # For each detected marker, treat its ID as the robot j
        for marker_corners, marker_id in zip(corners, ids_flat):
            j = int(marker_id)
            # Only consider valid robot IDs and skip self-detection
            if j < 1 or j > self.num_agents or j == agent_id:
                continue

            pts = marker_corners.reshape(-1, 2)
            u = float(np.mean(pts[:, 0]))  # pixel x

            # Horizontal bearing in robot i camera frame
            alpha = math.atan2(u - cx, fx)

            self.sigma_rel[(agent_id, j)] = alpha
            # You can uncomment if you want spammy logs:
            # self.get_logger().info(
            #     f"sigma_{agent_id}{j} (camera frame) = {alpha:.3f} rad"
            # )

    # ----------------- Marker publishing -----------------

    # def publish_b_marker_ij(self, i: int, j: int):
    #     """
    #     Publish a Marker arrow in map frame for b_ij (world-frame unit vector
    #     from robot i toward robot j).
    #     For visualization, we only do (1,2) here, but the math is general.
    #     """
    #     key = (i, j)
    #     if key not in self.b_ij or i not in self.positions:
    #         return

    #     start = self.positions[i]
    #     direction = self.b_ij[key]

    #     marker = Marker()
    #     marker.header.frame_id = "map"  # set RViz fixed frame to 'map'
    #     marker.header.stamp = self.get_clock().now().to_msg()
    #     marker.ns = f"b_{i}{j}"
    #     marker.id = i * 10 + j
    #     marker.type = Marker.ARROW
    #     marker.action = Marker.ADD

    #     marker.points.append(
    #         Point(x=float(start[0]), y=float(start[1]), z=0.2)
    #     )

    #     scale = 0.7
    #     end = start + scale * direction
    #     marker.points.append(
    #         Point(x=float(end[0]), y=float(end[1]), z=0.2)
    #     )

    #     marker.scale.x = 0.1
    #     marker.scale.y = 0.2
    #     marker.scale.z = 0.2

    #     marker.color.r = 1.0
    #     marker.color.g = 0.0
    #     marker.color.b = 0.0
    #     marker.color.a = 1.0

    #     self.marker_pub.publish(marker)
    
    # def beta_dot_world_ivp(self, t: float, beta: np.ndarray, be_i: np.ndarray, yaw_i: float) -> np.ndarray:
    #     """
    #     World-frame Step-3 dynamics:
    #     beta_dot = -rho * (phi phi^T) be_i - (phi_perp phi_perp^T) beta
    #     """
    #     phi = np.array([math.cos(yaw_i), math.sin(yaw_i)], dtype=float)
    #     phi_perp = np.array([-math.sin(yaw_i), math.cos(yaw_i)], dtype=float)

    #     P = np.outer(phi, phi)               # 2x2
    #     Pperp = np.outer(phi_perp, phi_perp) # 2x2

    #     beta = beta.reshape(2,)
    #     return (-rho * (P @ be_i) - (Pperp @ beta)).reshape(2,)


    # def integrate_beta(self, i: int, be_i: np.ndarray, yaw_i: float, dt: float) -> np.ndarray:
    #     """
    #     Integrate beta_i over one control period dt using solve_ivp.
    #     """
    #     beta0 = self.betai[i].astype(float)

    #     sol = solve_ivp(
    #         fun=self.beta_dot_world_ivp,
    #         t_span=(0.0, dt),
    #         y0=beta0,
    #         args=(be_i, yaw_i),
    #         method="RK45",
    #         max_step=dt,
    #         rtol=1e-6,
    #         atol=1e-8,
    #     )

    #     if not sol.success:
    #         self.get_logger().warn(f"solve_ivp failed for robot {i}: {sol.message}")
    #         return beta0

    #     return sol.y[:, -1]

    
    # ----------------- Timer: compute all b_ij and beta_ij -----------------

    def timer_callback(self):
        #dt = self.period
        if self.num_agents < 2:
            return

        for i in range(1, self.num_agents + 1):
            if i not in self.yaws:
                continue

            yaw_i = float(self.yaws[i])

            # # Heading unit vectors in WORLD
            # phi = np.array([math.cos(yaw_i), math.sin(yaw_i)], dtype=float)
            # phi_perp = np.array([-math.sin(yaw_i), math.cos(yaw_i)], dtype=float)

            # Rotation BODY -> WORLD (for bearing direction)
            R_plus = np.array(
                [[math.cos(yaw_i), -math.sin(yaw_i)],
                [math.sin(yaw_i),  math.cos(yaw_i)]],
                dtype=float
            )

            # # Build b_ei in WORLD frame
            # be_i = np.zeros(2, dtype=float)

            for j in sorted(self.adj.get(i, [])):
                key = (i, j)

                # need bearing sigma_ij from vision
                if key not in self.sigma_rel:
                    continue

                sigma_ij = float(self.sigma_rel[key])

                # bearing unit vector in BODY (camera frame assumed aligned with robot body x-axis)
                b_ij_body = np.array([math.cos(sigma_ij), math.sin(sigma_ij)], dtype=float)

                # convert to WORLD
                b_ij_world = R_plus @ b_ij_body
                self.b_ij[key] = b_ij_world

            #     # desired bearing from YAML initial positions (constant)
            #     if (i not in self.positions) or (j not in self.positions):
            #         continue

            #     d = self.positions[j] - self.positions[i]
            #     norm_d = np.linalg.norm(d)
            #     if norm_d < 1e-6:
            #         continue
            #     b_ij_star = d / norm_d

            #     be_i += (b_ij_world - b_ij_star)

            # # --- Step 3: integrate beta_i (WORLD frame) ---
            # self.betai[i] = self.integrate_beta(i, be_i, yaw_i, dt)

            # # --- Step 4: compute control in WORLD frame using dot products ---
            # v = kp * float(phi @ be_i) + ki * float(phi @ self.betai[i])
            # w = kw * float(phi_perp @ be_i) + ki * float(phi_perp @ self.betai[i])

            # # Publish cmd_vel
            # msg = Twist()
            # msg.linear.x = float(v)
            # msg.angular.z = float(w)
            # self.pubs[i].publish(msg)

        # # Visualization example
        # self.publish_b_marker_ij(1, 2)





def main():
    rclpy.init()
    node = CameraBearing()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
