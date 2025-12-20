import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pose_fusion.circle_estimate import estimate_3d_circle_center

def generate_circle_3d(center, normal, radius, n_points, noise_cov):
    normal = normal / np.linalg.norm(normal)
    arbitrary = np.array([1, 0, 0]) if not np.allclose(normal, [1, 0, 0]) else np.array([0, 1, 0])
    u = np.cross(normal, arbitrary)
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)

    angles = np.linspace(0, 2 * np.pi, n_points)
    circle_points = np.array([
        center + radius * np.cos(a) * u + radius * np.sin(a) * v
        for a in angles
    ])

    noise = np.random.multivariate_normal(mean=[0, 0, 0], cov=noise_cov, size=n_points)
    return circle_points + noise

def visualize(points, estimated_center, true_center):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], alpha=0.5, label='Noisy Samples')
    ax.scatter(*true_center, color='green', s=100, label='True Center')
    ax.scatter(*estimated_center, color='red', s=100, label='Estimated Center')
    ax.legend()
    ax.set_title('3D Circle Center Estimation')
    plt.show()

# Parameters
n_samples = 100
true_center = np.array([2.0, -3.0, 5.0])
true_normal = np.array([1.0, 2.0, 3.0])
true_radius = 4.0
noise_covariance = np.diag([0.05, 0.05, 0.05])

# Run
points_3d = generate_circle_3d(true_center, true_normal, true_radius, n_samples, noise_covariance)

print(points_3d)
estimated_center = estimate_3d_circle_center(points_3d)
# print(estimated_center)
visualize(points_3d, estimated_center, true_center)
