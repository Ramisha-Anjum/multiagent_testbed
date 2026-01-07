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


# Gains (tune later)
kp = 2.0
kw = 1.0
ki = 0.2
# t1 = 0

class PositionbasedTarget(Node):
    def __init__(self):
        super().__init__("positionbased_target")

        # ---------------- Params & config ----------------
        self.declare_parameter("config_file", "config1.yaml")
        self.declare_parameter('pose_topic_template', '/agent{}/pose')  # PoseStamped per robot

        pkg_share = get_package_share_directory("turtlesim_vicon")
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
            # self.adj[j].add(i)

        
        # ---------------- State storage ----------------
        # World positions for each robot
        self.positions: Dict[int, np.ndarray] = {}

        # Positions of each robots to calculate b_ij_star from config      
        position = net.get("position", [])
        for i, p in enumerate(position, start=1):
            x = float(p["x"])
            y = float(p["y"])
            self.positions[i] = np.array([x, y], dtype=float)
            self.get_logger().info(
            f"Positions to calculate b_ij_star from config: positions={self.positions}")

            # for j in sorted(self.adj.get(i, [])):

            #     d = self.positions[j] - self.positions[i]
            #     norm_d = np.linalg.norm(d)
            #     if norm_d < 1e-6:
            #         continue
            #     self.b_ij_star = d / norm_d
        
        # Pose storage: (x, y) from PoseStamped
        self.poses: Dict[int, Tuple[float, float, float]] = {}
        self.measurement = {}

        # World-frame unit vectors from i to j:
        # self.b_ij: Dict[Tuple[int, int], Tuple[float, float, float]] = {}
        # Bearings dictionary: b_ij[i][j] = [bx, by]
        self.b_ij: Dict[int, Dict[int, list]] = {}


    
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
            topic = f'/agent{i}/neighbors'
            self.posebearingsubs.append(
                self.create_subscription(RelativeNeighbors, topic, lambda msg, i=i: self.posebearing_callback(i, msg), 10)
            )

        # self.neighborsubs = []
        # for i in range(1, self.num_agents + 1):
        #     topic = f'/agent{i}/neighbors'
        #     self.neighborsubs.append(
        #         self.create_subscription(RelativeNeighbors, topic, lambda msg, i=i: self.neighbor_callback(i, msg), 10)
        #     )
        
        # Publishers
        self.pubs: Dict[int, rclpy.publisher.Publisher] = {}
        for i in range(1, self.num_agents + 1):
            topic = f'/turtle{i}/cmd_vel'
            self.pubs[i] = self.create_publisher(Twist, topic, 10)
        

        
        self.betai: Dict[int, np.ndarray] = {i: np.zeros(2, dtype=float) for i in range(1, self.num_agents + 1)}
        self.betai_dot: Dict[int, np.ndarray] = {i: np.zeros(2, dtype=float) for i in range(1, self.num_agents + 1)}

        # ---------------- Timer ----------------
        self.period = 0.05  # 20 Hz
        self.timer = self.create_timer(self.period, self.timer_callback)

        self.get_logger().info("positionbased_target node started.")


    def pose_callback(self, agent_id: int, msg: PoseStamped):
        # Assume z = 0, extract x,y
        # For orientation of the robot
        quat = msg.pose.orientation
        w, x, y, z = quat.w, quat.x, quat.y, quat.z
        
        q_real = 1 - 2 * (y*y + z*z)
        q_imag = 2 * (x*y + w*z)
        norm = np.hypot(q_real, q_imag)
        if norm == 0.0:
            q_real, q_imag = 1.0, 0.0
        else:
            q_real /= norm; q_imag /= norm
        theta = math.atan2(q_imag, q_real)
        # storing poses for each agent
        self.poses[agent_id] = (float(msg.pose.position.x), float(msg.pose.position.y), theta)

    # def neighbor_callback(self, agent_id: int, msg: RelativeNeighbors):
    #     self.measurement[agent_id] = {}
    #     for n_j in msg.neighbors:
    #         self.measurement[agent_id][n_j.neighbor_id] = [float(n_j.relative_position.x), float(n_j.relative_position.y)]


    
    def posebearing_callback(self, agent_id: int, msg: RelativeNeighbors):
        self.b_ij[agent_id] = {}
        for n_j in msg.neighbors:
            self.b_ij[agent_id][n_j.neighbor_id] = [float(n_j.relative_bearing.x), float(n_j.relative_bearing.y)]

    
    # def beta_dot_world_ivp(self, beta, be_i, yaw_i):
    #     """
    #     World-frame Step-3 dynamics:
    #     beta_dot = -rho * (phi phi^T) be_i - (phi_perp phi_perp^T) beta
    #     """
    #     phi = np.array([math.cos(yaw_i), math.sin(yaw_i)], dtype=float)
    #     phi_perp = np.array([-math.sin(yaw_i), math.cos(yaw_i)], dtype=float)

    #     p = np.outer(phi, phi)               # 2x2
    #     Pperp = np.outer(phi_perp, phi_perp) # 2x2

    #     beta = beta.reshape(2,)
    #     return ((p @ be_i) - (Pperp @ beta))


    # def integrate_beta(self, i, be_i, yaw_i, dt) -> np.ndarray:
    #     """
    #     Integrate beta_i over one control period dt using solve_ivp.
    #     """
    #     beta0 = self.betai[i].astype(float)

    #     # sol = solve_ivp(
    #     #     fun=self.beta_dot_world_ivp,
    #     #     t_span=(0.0, dt),
    #     #     y0=beta0,
    #     #     args=(be_i, yaw_i),
    #     #     method="RK45",
    #     #     max_step=dt,
    #     #     rtol=1e-6,
    #     #     atol=1e-8,
    #     # )
    #     sol = solve_ivp(
    #     fun=lambda t, y: self.beta_dot_world_ivp(y, be_i, yaw_i),
    #     t_span=(t1, dt),
    #     y0=beta0,
    #     method="RK45",
    #     max_step=dt,
    #     rtol=1e-6,
    #     atol=1e-8,
    #     )
        
    #     if not sol.success:
    #         self.get_logger().warn(f"solve_ivp failed for robot {i}: {sol.message}")
    #         return beta0

    #     return sol.y[:, -1]

    
    # ----------------- Timer: compute all b_ij and beta_ij -----------------

    def timer_callback(self):
        dt = self.period

        # Must have poses and bearings for all agents
        for i in range(1, self.num_agents + 1):
            if i not in self.poses:
                return
            if i not in self.b_ij:
                return

        # beta = np.zeros((self.num_agents, 2), dtype=float)

        for i in range(1, self.num_agents + 1):
            # idx = i - 1

            yaw_i = self.poses[i][2]
            phi = np.array([math.cos(yaw_i), math.sin(yaw_i)], dtype=float)
            phi_perp = np.array([-math.sin(yaw_i), math.cos(yaw_i)], dtype=float)

            p = np.outer(phi, phi)
            Pperp = np.outer(phi_perp, phi_perp)

            be_i = np.zeros(2, dtype=float)

            for j in sorted(self.adj.get(i, [])):
                if j not in self.b_ij[i]:
                    continue

                b_ij_world = np.array(self.b_ij[i][j], dtype=float)



                d = self.positions[j] - self.positions[i]
                norm_d = np.linalg.norm(d)
                if norm_d < 1e-6:
                    continue
                b_ij_star = d / norm_d

                be_i += (b_ij_world - b_ij_star)
                self.get_logger().info(f"Bearing Error between robot i={i} and robot j={j} is ={be_i}")

            # Step 3 integrate beta_i
            # self.betai[i] = self.integrate_beta(i, be_i, yaw_i, dt)

            self.betai[i] = self.betai[i] + self.betai_dot[i]

            self.betai_dot[i] = (p @ be_i) - (Pperp @ self.betai[i])
            self.get_logger().info(f"for robot t1 = 0i={i}, beta_i={self.betai[i]}")

            # beta_i = (p @ be_i) - (Pperp @ self.betai[i])
            # beta[idx] = beta_i

            # Step 4 control (scalar projections)
            v = (kp * (phi @ be_i)) + (ki * (phi @ self.betai[i]))
            w = (kw * (phi_perp @ be_i)) + (ki * (phi_perp @ self.betai[i]))

            cmd = Twist()
            cmd.linear.x = float(v)
            cmd.angular.z = float(w)
            self.pubs[i].publish(cmd)
            # t1 = t1 + dt


    # def timer_callback(self):
    #     dt = self.period
    #     if self.num_agents < 2:
    #         return

    #     beta = np.zeros((self.num_agents,2))
    #     for i in range(1, self.num_agents + 1):

    #         # Heading unit vectors in WORLD
    #         phi = np.array([math.cos(self.poses[i][2]), math.sin(self.poses[i][2])], dtype=float)
    #         phi_perp = np.array([-math.sin(self.poses[i][2]), math.cos(self.poses[i][2])], dtype=float)
    #         p = np.outer(phi, phi)                  # 2x2
    #         Pperp = np.outer(phi_perp, phi_perp)    # 2x2

    #         # Build b_ei in WORLD frame
    #         be_i = np.zeros(2, dtype=float)

    #         for j in sorted(self.adj.get(i, [])):

    #             b_ij_world = self.b_ij [i][j] 

    #             # desired bearing from YAML initial positions (constant)
    #             if (i not in self.positions) or (j not in self.positions):
    #                 continue

    #             d = self.positions[j] - self.positions[i]
    #             norm_d = np.linalg.norm(d)
    #             if norm_d < 1e-6:
    #                 continue
    #             b_ij_star = d / norm_d

    #             be_i += (b_ij_world - b_ij_star)

    #         # --- Step 3: integrate beta_i (WORLD frame) ---
    #         self.betai[i] = self.integrate_beta(i, be_i, self.poses[i][2], dt)

    #         beta[i] = p @ be_i - Pperp @ self.betai[i]

    #         # --- Step 4: compute control in WORLD frame using dot products ---
    #         v = (kp * phi.T @ be_i) + (ki * phi.T @ beta[i])
    #         w = (kw * phi_perp.T @ be_i) + (ki * phi_perp.T @ beta[i])

    #         # Publish cmd_vel
    #         msg = Twist()
    #         msg.linear.x = float(v)
    #         msg.angular.z = float(w)
    #         self.pubs[i].publish(msg)





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
