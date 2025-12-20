import numpy as np

def fit_plane(points):
    centroid = points.mean(axis=0)
    centered = points - centroid
    _, _, Vt = np.linalg.svd(centered)
    normal = Vt[2, :]
    return centroid, Vt[:2, :], normal

def compute_rotation_matrix_to_z(normal):
    z_axis = np.array([0, 0, 1])
    normal = normal / np.linalg.norm(normal)
    v = np.cross(normal, z_axis)
    c = np.dot(normal, z_axis)
    if np.allclose(v, 0):
        return np.eye(3)
    vx = np.array([[0, -v[2], v[1]],
                   [v[2], 0, -v[0]],
                   [-v[1], v[0], 0]])
    R = np.eye(3) + vx + vx @ vx * (1 / (1 + c))
    return R

def fit_circle_2d(points_2d):
    x, y = points_2d[:, 0], points_2d[:, 1]
    A = np.c_[2 * x, 2 * y, np.ones_like(x)]
    b = x**2 + y**2
    cx, cy, _ = np.linalg.lstsq(A, b, rcond=None)[0]
    return np.array([cx, cy])

def estimate_3d_circle_center(points):
    """
    Estimate the 3D center of a noisy tilted circle embedded in 3D space.

    Parameters:
        points (np.ndarray): Nx3 array of 3D points.

    Returns:
        np.ndarray: Estimated 3D coordinates of the circle center.
    """
    centroid, _, normal = fit_plane(points)
    R = compute_rotation_matrix_to_z(normal)
    points_flat = (points - centroid) @ R.T
    center_2d = fit_circle_2d(points_flat[:, :2])
    center_3d = center_2d @ R[:2, :] + centroid
    return center_3d
