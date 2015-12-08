from math import *
import numpy as np
import kin_functions as kfs
from geometry_msgs.msg import TransformStamped

def quaternion_to_exp(rot):
    """
    Converts a quaternion vector in 3D to its corresponding omega and theta.
    This uses the quaternion -> exponential coordinate equation given in Lab 6
    
    Args:
    rot - a (4,) nd array or 4x1 array: the quaternion vector (\vec{q}, q_o)
    
    Returns:
    omega - (3,) ndarray: the rotation vector
    theta - a scalar
    """
    theta = 2.0 * np.arccos(rot[-1])
    if theta == 0:
        omega = np.array([0.0, 0.0, 0.0])
    else:
        omega = ((1.0/sin(theta/2.0)) * rot[:-1])
    
    return (omega, theta)
    
def create_rbt(omega, theta, p):
    """
    Creates a rigid body transform using omega, theta, and the translation component.
    g = [R,p; 0,1], where R = exp(omega * theta), p = trans
    
    Args:
    omega - (3,) ndarray : the axis you want to rotate about
    theta - scalar value
    trans - (3,) ndarray or 3x1 array: the translation component of the rigid body motion
    
    Returns:
    g - (4,4) ndarray : the rigid body transform
    """
    R = kfs.rotation_3d(omega, theta)
    g = np.array([[R[0][0], R[0][1], R[0][2], p[0]], [R[1][0], R[1][1], R[1][2], p[1]], [R[2][0], R[2][1], R[2][2], p[2]], [0, 0, 0, 1]])
    
    return g
    
def compute_gab(g0a,g0b):
    """
    Creates a rigid body transform g_{ab} the converts between frame A and B
    given the coordinate frame A,B in relation to the origin
    
    Args:
    g0a - (4,4) ndarray : the rigid body transform from the origin to frame A
    g0b - (4,4) ndarray : the rigid body transform from the origin to frame B
    
    Returns:
    gab - (4,4) ndarray : the rigid body transform
    """
    return np.dot(np.linalg.inv(g0a),g0b)
    
def find_omega_theta(R):
    """
    Given a rotation matrix R, finds the omega and theta such that R = exp(omega * theta)
    
    Args:
    R - (3,3) ndarray : the rotational component of the rigid body transform
    
    Returns:
    omega - (3,) ndarray : the axis you want to rotate about
    theta - scalar value
    """
    theta = np.arccos((np.trace(R) - 1)/2)
    omega = (1/(2*sin(theta)))*np.array([R[2][1] - R[1][2],R[0][2] - R[2][0],R[1][0] - R[0][1]])
    return omega, theta
    
def find_v(omega, theta, trans):
    """
    Finds the linear velocity term of the twist (v,omega) given omega, theta and translation
    
    Args:
    omega - (3,) ndarray : the axis you want to rotate about
    theta - scalar value
    trans - (3,) ndarray of 3x1 list : the translation component of the rigid body transform
    
    Returns:
    v - (3,1) ndarray : the linear velocity term of the twist (v,omega)
    """    
    A_1 = np.eye(3) - kfs.rotation_3d(omega, theta)
    A_1 = A_1.dot(kfs.skew_3d(omega))
    A_2 = np.outer(omega, omega.T)*theta
    A = A_1 + A_2
    v = np.dot(np.linalg.inv(A), trans)
    return np.array([v]).T

def gab_to_ts(gab):
    return TransformStamped()