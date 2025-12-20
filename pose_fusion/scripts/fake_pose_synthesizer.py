from pose_fusion.spatial import Transformation
from pose_fusion.conversion import euler_to_quaternion
from pose_fusion.utils import plot_frames

import pylgmath
import numpy as np

# create nominal transformation 

orientation = euler_to_quaternion(np.pi/2,0,-np.pi/2)
position = [1,2,-3]
T_mean = Transformation(rotation=orientation,translation=position)

# generate random perturbation in Lie Algebra

cov = np.diag([0.01,0.01,0.01,0.05,0.05,0.05])
num = 20
perturbations = np.random.multivariate_normal(np.zeros(6),cov,(num))

# apply each perturbation to the nominal one

transforms = []
for perturb in perturbations:
    T_noise = pylgmath.Transformation(xi_ab=np.array([perturb]).T)
    T_noisy = T_mean @ T_noise
    transforms.append(T_noisy.matrix())
plot_frames(transforms)