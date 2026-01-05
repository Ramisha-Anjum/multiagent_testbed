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

from geometry_msgs.msg import PoseStamped, Twist
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Image, CameraInfo, Imu
# from visualization_msgs.msg import Marker
from relative_position_msg.msg import RelativeNeighbors, RelativeMeasurement


# import cv2
# from cv_bridge import CvBridge


# # Gains (tune later)
# kp = 0.8
# kw = 1.5
# ki = 0.2
# rho = 1.0

class PositionbasedTarget(Node):
    def __init__(self):
        super().__init__("positionbased_target")

        # ---------------- Params & config ----------------
        self.declare_parameter("config_file", "config.yaml")
        self.declare_parameter('pose_topic_template', '/robot{}/pose')  # PoseStamped per robot

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
        
        # Pose storage: (x, y) from PoseStamped
        self.poses: Dict[int, Tuple[float, float, float, float]] = {}
        self.measurement = {}

        # World-frame unit vectors from i to j:
        self.b_ij: Dict[Tuple[int, int], Tuple[float, float, float, float]] = {}
    
        # Subscriptions
        pose_topic_template = self.get_parameter('pose_topic_template').get_parameter_value().string_value
        self.posesubs = []
        for i in range(1, self.num_agents + 1):
            topic = pose_topic_template.format(i)
            self.posesubs.append(
                self.create_subscription(PoseStamped, topic, lambda msg, i=i: self.pose_callback(i, msg), 10)
            )
        
        self.posebearingsubs = []
        for i in range(1, self.num_agents + 1):
            topic = f'/robot{i}/neighbors'
            self.posebearingsubs.append(
                self.create_subscription(RelativeNeighbors, topic, lambda msg, i=i: self.posebearing_callback(i, msg), 10)
            )

        # self.neighborsubs = []
        # for i in range(1, self.num_agents + 1):
        #     topic = f'/robot{i}/neighbors'
        #     self.neighborsubs.append(
        #         self.create_subscription(RelativeNeighbors, topic, lambda msg, i=i: self.neighbor_callback(i, msg), 10)
        #     )
        
        # Publishers
        self.pubs: Dict[int, rclpy.publisher.Publisher] = {}
        for i in range(1, self.num_agents + 1):
            topic = f'/turtle{i}/cmd_vel'
            self.pubs[i] = self.create_publisher(Twist, topic, 10)
        

        # # beta_ij[(i, j)] = b_ij expressed in robot i body frame (optional)
        # #self.beta_ij: Dict[Tuple[int, int], np.ndarray] = {}
        # self.betai: Dict[int, np.ndarray] = {i: np.zeros(2, dtype=float) for i in range(1, self.num_agents + 1)}

        # ---------------- Timer ----------------
        self.period = 0.05  # 20 Hz
        self.timer = self.create_timer(self.period, self.timer_callback)

        self.get_logger().info("positionbased_target node started.")


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

    def pose_callback(self, agent_id: int, msg: PoseStamped):
        # Assume z = 0, extract x,y
        # Projecting x axis to 2D for orientation vector
        quat = msg.pose.orientation
        w, x, y, z = quat.w, quat.x, quat.y, quat.z
        
        q_real = 1 - 2 * (y*y + z*z)
        q_imag = 2 * (x*y + w*z)
        norm = np.hypot(q_real, q_imag)
        if norm == 0.0:
            q_real, q_imag = 1.0, 0.0
        else:
            q_real /= norm; q_imag /= norm
        # storing poses for each agent
        self.poses[agent_id] = (float(msg.pose.position.x), float(msg.pose.position.y), q_real,q_imag)

    # def neighbor_callback(self, robot_id: int, msg: RelativeNeighbors):
    #     self.measurement[robot_id] = {}
    #     for n_j in msg.neighbors:
    #         self.measurement[robot_id][n_j.neighbor_id] = [float(n_j.relative_position.x), float(n_j.relative_position.y)]


    
    def posebearing_callback(self, robot_id: int, msg: RelativeNeighbors):
        self.measurement[robot_id] = {}
        for n_j in msg.neighbors:
            self.measurement[robot_id][n_j.neighbor_id] = [float(n_j.relative_bearing.x), float(n_j.relative_bearing.y)]

    
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





def main():
    rclpy.init()
    node = PositionbasedTarget()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
