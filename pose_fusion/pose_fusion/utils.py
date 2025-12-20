#!/usr/bin/python3
import numpy as np
from typing import List
import pylgmath
import matplotlib.pyplot as plt

from pose_fusion.spatial import Transformation

class PoseDataSynthesizer():
    @staticmethod
    def synthesize(nominal_pose: np.ndarray, covariance: np.ndarray, num: int = 1):
        """
        Synthesize pose data with perturbations.

        Parameters:
            nominal_pose (np.ndarray): Nominal pose as 4x4 homogeneous transformation matrix.
            covariance (np.ndarray): Covariance matrix for the perturbations.
            num (int, optional): Number of datasets to generate. Defaults to 1.

        Returns:
            transforms (List[np.ndarray]): List of 4x4 homogeneous transformation matrices.
            covariance_matrices (List[np.ndarray]): List of covariance matrices.

        Raises:
            ValueError: If nominal_pose or covariance are not 4x4 or 6x6 matrices, respectively.
        """
        if nominal_pose.shape != (4, 4):
            raise ValueError('nominal_pose must be a 4x4 matrix.')
        if covariance.shape != (6, 6):
            raise ValueError('covariance must be a 6x6 matrix.')

        ground_truth = Transformation(transformation=nominal_pose)
        covariance_matrices = [covariance] * num
        perturbations = np.random.multivariate_normal(np.zeros(6), covariance, (num))
        transforms = [(ground_truth @ pylgmath.Transformation(xi_ab=np.array([perturb]).T)).matrix() for perturb in perturbations]

        return transforms, covariance_matrices
class TransformDataSet(List):
    """
    A list of `TransformDataPoint` objects representing a dataset of 3D transformations and their corresponding
    covariance matrices.

    """
    class TransformDataPoint():
        """
        A class representing a single data point in a `TransformDataSet` object.
        """
        
        def __init__(self,transform:pylgmath.Transformation,covariance:np.ndarray):
            """
            Initialize a new `TransformDataPoint` object.

            Args:
                transform (pylgmath.Transformation): The transformation matrix of the data point.
                covariance (np.ndarray): The covariance matrix of the data point.
            """
            self.transform = transform
            self.covariance = covariance
    def __init__(
            self,
            transform_dataset:List[np.ndarray],
            covariance_dataset:List[np.ndarray]
        )->List[TransformDataPoint]:
        """
        Initialize a new `TransformDataSet` object.

        Args:
            transform_dataset (List[np.ndarray]): A list of 4x4 transformation matrices representing the
                ground truth poses in the dataset.
            covariance_dataset (List[np.ndarray]): A list of 6x6 covariance matrices representing the
                uncertainty in each ground truth pose.
        """
        data_set = []
        for i in range(len(transform_dataset)):
            data_point = self.TransformDataPoint(
                transform=pylgmath.Transformation(T_ba=transform_dataset[i]),
                covariance=covariance_dataset[i]
            )
            data_set.append(data_point)
        super().__init__(data_set)
def plot_frames(transforms):
    # Example list of 4x4 homogeneous transformation matrices representing poses in 3D
    poses_list = transforms  # Your list of 4x4 matrices here

    # Extract the translation vectors and rotation matrices from the homogeneous transformation matrices
    translations = np.array([pose[:3, 3] for pose in poses_list])
    rotations = np.array([pose[:3, :3] for pose in poses_list])

    # Define a scaling factor for the arrow length
    scaling_factor = 0.1  # Modify this value to adjust the arrow length

    # Scale the rotation matrices by the scaling factor
    rotations_scaled = rotations * scaling_factor

    # Extract the x, y, z coordinates from the translation vectors
    x = translations[:, 0]
    y = translations[:, 1]
    z = translations[:, 2]

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the frames as arrows with scaled lengths
    for i in range(len(x)):
        ax.quiver(x[i], y[i], z[i], rotations_scaled[i][0, 0], rotations_scaled[i][1, 0], rotations_scaled[i][2, 0], color='red')
        ax.quiver(x[i], y[i], z[i], rotations_scaled[i][0, 1], rotations_scaled[i][1, 1], rotations_scaled[i][2, 1], color='green')
        ax.quiver(x[i], y[i], z[i], rotations_scaled[i][0, 2], rotations_scaled[i][1, 2], rotations_scaled[i][2, 2], color='blue')

    # Set plot limits and labels
    ax.set_xlim([np.min(x), np.max(x)])
    ax.set_ylim([np.min(y), np.max(y)])
    ax.set_zlim([np.min(z), np.max(z)])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Show the plot
    plt.show()