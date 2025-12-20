#!/usr/bin/python3
import numpy as np
from typing import Literal, List, Union, Optional
import pylgmath
from pose_fusion.conversion import quaternion_to_euler, quaternion_to_matrix

class StandardSO3(pylgmath.Rotation):
    """
    This class represents a rotation in three-dimensional space that is constructed by rotating about one of the standard axes (x, y, or z).
    An SO3 object can be used to represent the rotation of a rigid body in 3D space.

    Methods:
        __init__(axis='z', angle=0):
            Initializes a StandardSO3 object with the specified axis and angle.

    """
    def __init__(self,axis:Literal['x','y','z']='z',angle:float=0):
        """
        Initializes a StandardSO3 object with the specified axis and angle.

        Args:
            axis (str, optional): The axis of rotation. Must be one of 'x', 'y', or 'z'. Defaults to ''.
            angle (float, optional): The angle of rotation in radians. Defaults to 0.
        """
        axes = {'x':0,'y':1,'z':2}
        tangent_vector = [0,0,0]
        if axis:
            tangent_vector[axes[axis]] = angle
        super().__init__(aaxis_ab=np.array([tangent_vector]).T)
class RPY(pylgmath.Rotation):
    """
    This class represents a rotation in three-dimensional space that is constructed based on given roll-pitch-yaw (RPY) angles.
    An RPY object can be used to represent the rotation of a rigid body in 3D space.

    Methods:
        __init__(angles=[0,0,0]):
            Initializes an RPY object with the specified roll-pitch-yaw angles.

    """
    def __init__(self,angles:List[object]=[0,0,0]):
        """
        Initializes an RPY object with the specified roll-pitch-yaw angles.

        Args:
            angles (list, optional): A list of three angles in radians, representing the roll, pitch, and yaw angles, respectively. Defaults to [0, 0, 0].
        """
        axes = ['x','y','z']
        R = StandardSO3()
        for angle,axis in zip(angles,axes):
            R = StandardSO3(axis=axis,angle=angle)@R
        super().__init__(rotation=R)
class Quaternion(pylgmath.Rotation):
    """
    This class represents a rotation in three-dimensional space using a quaternion.
    
    The quaternion is assumed to be in the format [w, x, y, z], where w is the scalar part
    and x, y, z are the vector part.
    
    The tangent vector in the Lie algebra of SO(3) is computed using the formula:
    t = 2 * angle * axis / norm(axis), where angle is the rotation angle and axis is
    the rotation axis.

    Methods:
        __init__(quat=[1.0, 0.0, 0.0, 0.0]):
            Initializes an Quaternion object with the specified quaternion.

    """
    def __init__(self, quat: List[float]=[1.0, 0.0, 0.0, 0.0]):
        """
        Create a Quaternion object from a quaternion.

        Args:
            quat (list, optional): A list of quaternion's element [w,x,y,z], respectively. Defaults to [1, 0, 0, 0].
        """
        R = np.array(quaternion_to_matrix(quat))
        super().__init__(C_ba=R)
class Translation(np.ndarray):
    """
    This class represents a translation in three-dimensional space.
    A Translation object can be used to represent the translation of a point in 3D space.

    Methods:
        __new__(translation_vector=[0,0,0]):
            Creates a new Translation object with the specified translation vector.

    """
    def __new__(self,translation:List[float]=[0,0,0]):
        """
        Creates a new Translation object with the specified translation vector.

        Args:
            translation_vector (list, optional): A list of three numbers representing the x, y, and z components of the translation vector. Defaults to [0, 0, 0].

        Returns:
            Translation: A new Translation object with the specified translation vector.
        """
        p = np.array([translation]).T
        return p
class Transformation(pylgmath.Transformation):
    """
    This class represents a transformation in three-dimensional space.

    Methods:
        __new__(rotation=None,translation=None,transformation=None):
            Creates a new Transformation object with the specified (rotation and/or translation vector) or transformation.

    """
    def __init__(
            self,
            rotation:Optional[Union[RPY,List[float]]]=None,
            translation:Optional[Union[Translation,List[float]]]=None,
            transformation:Optional[Union[pylgmath.Transformation,np.ndarray]]=None):
        
        """
        Create a 3D transformation that represents a rotation followed by a translation.

        Args:
            rotation: Optional. Specify the rotation component of the transformation.
                - If None, an identity rotation is used.
                - If a RPY object is provided, it is used directly as the rotation.
                - If a list of floats with length 3 or 4 is provided, it is interpreted as Euler angles
                  or quaternion representation of the rotation, respectively.
                  In the case of Euler angles, they are interpreted as roll, pitch, and yaw, in radians.
                  In the case of quaternions, they are converted to Euler angles before being used as the rotation.
                  See pose_fusion.conversion.quaternion_to_euler() for more details.
                - If an invalid type is provided, a TypeError is raised.

            translation: Optional. Specify the translation component of the transformation.
                - If None, an identity translation (zero vector) is used.
                - If a Translation object is provided, it is used directly as the translation.
                - If a list of floats with length 3 is provided, it is used as the translation vector.
                - If an invalid type is provided, a TypeError is raised.

            transformation: Optional. Specify the entire transformation matrix directly.
                - If None, the rotation and translation components are used to construct the transformation matrix.
                - If a pylgmath.Transformation object is provided, it is used directly as the transformation.
                - If a 4x4 numpy array is provided, it is used as the transformation matrix.
                - If an invalid type is provided, a TypeError is raised.

        Raises:
            TypeError: If an invalid type is provided for any argument.
                       If the transformation matrix is provided at the same time as the other arguments.
        """

        empty_flags = {'rotation':False,'translation':False,'transformation':False}
        
        if rotation is None:
            empty_flags['rotation'] = True
            R = Quaternion()
        elif isinstance(rotation,RPY) or isinstance(rotation,Quaternion):
            R = rotation
        elif isinstance(rotation,List):
            if len(rotation)==3: # if roll-pitch-yaw
                R = RPY(rotation)
            elif len(rotation)==4:
                R = RPY(angles=quaternion_to_euler(rotation))
                R = Quaternion(quat=rotation)
        else:
            raise TypeError('Invalid type for rotation')
        if translation is None:
            empty_flags['translation'] = True
            p = Translation()
        elif isinstance(translation,np.ndarray):
            p = translation  
        elif isinstance(translation,List):
            p = Translation(translation)
        else:
            raise TypeError('Invalid type for translation')
        if transformation is None:
            empty_flags['transformation'] = True
            T = pylgmath.Transformation(T_ba=np.eye(4))
        elif isinstance(transformation,pylgmath.Transformation):
            T = transformation
        elif isinstance(transformation,np.ndarray):
            T = pylgmath.Transformation(T_ba=transformation)
        else:
            raise TypeError('Invalid type for transformation')
        if ( (not empty_flags['rotation']) or (not empty_flags['translation'])) and (not empty_flags['transformation']):
            raise TypeError('Cannot in put transformation matrix at same time as the other arguments.')    
        elif not empty_flags['rotation'] or not empty_flags['translation']:
            Tm = np.vstack((np.hstack((R.matrix(),p)),[0,0,0,1]))
            T = pylgmath.Transformation(T_ba=Tm)
        super().__init__(transformation=T)