#!/usr/bin/python3

import math
from typing import List, Tuple

def rotvec_to_quaternion(rotvec: List[float]) -> List[float]:
    """
    Convert a rotation vector to a unit quaternion.

    Args:
        rotvec (List[float]): The rotation vector represented as a list of three
            float values [x, y, z].

    Returns:
        List[float]: A unit quaternion represented as a list of four float values
            [w, x, y, z], where w is the scalar component and x, y, z are the vector
            components.
    Raises:
        ValueError: If the input rotation matrix is not a valid rotation matrix.
    
    """
    norm = math.sqrt(rotvec[0]**2+rotvec[1]**2+rotvec[2]**2)
    if norm == 0:
        return [1, 0, 0, 0]
    else:
        theta = norm
        axis = rotvec / norm
        qw = math.cos(theta/2)
        qx = axis[0] * math.sin(theta/2)
        qy = axis[1] * math.sin(theta/2)
        qz = axis[2] * math.sin(theta/2)
        return [qw, qx, qy, qz]
def matrix_to_quaternion(R: List[List[float]]) -> List[float]:
    """
    Convert a rotation matrix to a unit quaternion.

    Args:
        R (List[List[float]]): The rotation matrix represented as a list of lists
            of float values.

    Returns:
        List[float]: A unit quaternion represented as a list of four float values
            [w, x, y, z], where w is the scalar component and x, y, z are the vector
            components.

    Raises:
        ValueError: If the input rotation matrix is not a valid rotation matrix.
    """
    
    q = [1,0,0,0]
    tr = R[0][0]+R[1][1]+R[2][2]

    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2.0
        q[0] = 0.25 * S
        q[1] = (R[2, 1] - R[1, 2]) / S
        q[2] = (R[0, 2] - R[2, 0]) / S
        q[3] = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        q[0] = (R[2, 1] - R[1, 2]) / S
        q[1] = 0.25 * S
        q[2] = (R[0, 1] + R[1, 0]) / S
        q[3] = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        q[0] = (R[0, 2] - R[2, 0]) / S
        q[1] = (R[0, 1] + R[1, 0]) / S
        q[2] = 0.25 * S
        q[3] = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        q[0] = (R[1, 0] - R[0, 1]) / S
        q[1] = (R[0, 2] + R[2, 0]) / S
        q[2] = (R[1, 2] + R[2, 1]) / S
        q[3] = 0.25 * S

    return q
def quaternion_to_matrix(q: List[float]) -> List[List[float]]:
    """
    Convert a unit quaternion to a rotation matrix.

    Args:
        q (List[float]): A unit quaternion represented as a list of four float values
            [w, x, y, z], where w is the scalar component and x, y, z are the vector
            components.

    Returns:
        List[List[float]]: The rotation matrix represented as a list of lists of
            float values.

    Raises:
        ValueError: If the input quaternion is not a unit quaternion.
    """
    R = [
        [1 - 2 * q[2]**2 - 2 * q[3]**2, 2 * q[1]*q[2] - 2 * q[0]*q[3], 2 * q[1]*q[3] + 2 * q[0]*q[2]],
        [2 * q[1]*q[2] + 2 * q[0]*q[3], 1 - 2 * q[1]**2 - 2 * q[3]**2, 2 * q[2]*q[3] - 2 * q[0]*q[1]],
        [2 * q[1]*q[3] - 2 * q[0]*q[2], 2 * q[2]*q[3] + 2 * q[0]*q[1], 1 - 2 * q[1]**2 - 2 * q[2]**2]
    ]
    return R
def quaternion_to_euler(q: List[float]) -> Tuple[float, float, float]:
    """
    Convert a unit quaternion to Euler angles (roll, pitch, yaw).

    Args:
        q (List[float]): A unit quaternion represented as a list of four float values
            [w, x, y, z], where w is the scalar component and x, y, z are the vector
            components.

    Returns:
        Tuple[float, float, float]: The Euler angles (roll, pitch, yaw) represented
            as a tuple of float values.

    Raises:
        ValueError: If the input quaternion is not a unit quaternion.
    """
    q = [q_e/ math.sqrt(q[0]**2+q[1]**2+q[2]**2+q[3]**2) for q_e in q]  # normalize quaternion
    qw, qx, qy, qz = q

    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> List[float]:
    """
    Convert Euler angles (roll, pitch, yaw) to a unit quaternion.

    Args:
        roll (float): The roll angle in radians.
        pitch (float): The pitch angle in radians.
        yaw (float): The yaw angle in radians.

    Returns:
        List[float]: A unit quaternion represented as a list of four float values
            [w, x, y, z], where w is the scalar component and x, y, z are the vector
            components.
    """
    cr, cp, cy = math.cos(roll/2), math.cos(pitch/2), math.cos(yaw/2)
    sr, sp, sy = math.sin(roll/2), math.sin(pitch/2), math.sin(yaw/2)

    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy

    return [qw, qx, qy, qz]
