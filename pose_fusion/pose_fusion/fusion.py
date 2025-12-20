#!/usr/bin/python3
import numpy as np
from typing import List, Tuple
import pylgmath

from pysteam.evaluable import se3 as se3ev
from pysteam.evaluable.se3 import se3_error
from pysteam import problem, solver

from pose_fusion.utils import TransformDataSet

class PoseFusion():
    """
    A class for fusing multiple 3D poses using nonlinear optimization techniques.

    Attributes:
        transform_dataset (List[np.ndarray]): A list of numpy arrays representing the transformation matrices of each pose.
        covariance_dataset (List[np.ndarray]): A list of numpy arrays representing the covariance matrices of each pose.

    Methods:
        fuse_poses(method: str = "man_opt") -> Tuple[np.ndarray, np.ndarray]:
            Fuses the poses in the dataset using the specified method.

            Args:
                method (str, optional): The fusion method to use. Currently supported methods are:
                    - "man_opt": Uses the Manifold Optimization technique to find the optimal fused pose. (default)
                    - "avg_tan_vec": Uses the Average Tangent Vector technique to find the naive fused pose.

            Returns:
                A tuple containing:
                - The fused pose as a numpy array of shape (4, 4)
                - The covariance matrix of the fused pose as a numpy array of shape (6, 6)
    """
    def __init__(
            self,
            transform_dataset:List[np.ndarray],
            covariance_dataset:List[np.ndarray]
        ):
        self.data_set = TransformDataSet(
            transform_dataset=transform_dataset,
            covariance_dataset=covariance_dataset
        )
        
        self.fused_pose = pylgmath.Transformation()
        # Set up state variable
        T_var = se3ev.SE3StateVar(self.fused_pose )  # wrap as a state variable

        ## Set up loss function
        loss_func = problem.L2LossFunc()

        ## Setup cost terms
        cost_terms = []
        for i in range(len(self.data_set)):
            # Set up error
            error_func = se3_error(T_var,self.data_set[i].transform)
            # Set up noise
            noise_model = problem.StaticNoiseModel(self.data_set[i].covariance)
            cost_terms.append(
                problem.WeightedLeastSquareCostTerm(error_func, noise_model, loss_func)
            )

        ## Make optimization problem
        opt_prob = problem.OptimizationProblem()
        opt_prob.add_state_var(T_var)
        opt_prob.add_cost_term(*cost_terms)
        ## Make solver and solve
        self.optimizer = solver.GaussNewtonSolver(opt_prob, verbose=False)
        self.opt_prob = opt_prob
        
    def fuse_poses(self,method:str="man_opt")->Tuple[np.ndarray, np.ndarray]:
        """
        Fuses the poses in the dataset using the specified method.

        Args:
            method (str, optional): The fusion method to use. Currently supported methods are:
                - "man_opt": Uses the Manifold Optimization technique to find the optimal fused pose. (default)
                - "avg_tan_vec": Uses the Average Tangent Vector technique to find the optimal fused pose.

        Returns:
            A tuple containing:
            - The fused pose as a numpy array of shape (4, 4)
            - The covariance matrix of the fused pose as a numpy array of shape (6, 6)
        """
        if method =="man_opt":
            self.optimizer.optimize()
            hessian,vector = self.opt_prob.build_gauss_newton_terms()
            fused_pose = self.fused_pose.matrix()
            cov = np.linalg.inv(hessian)
        elif method =="avg_tan_vec":
            les = []
            for data in self.data_set:
                les.append(data.transform.vec().T[0])
            mean = np.mean(np.array(les),0)
            fused_pose = pylgmath.Transformation(xi_ab=np.array([mean]).T).matrix()
            cov = np.cov(np.array(les).T)
        return fused_pose,cov

