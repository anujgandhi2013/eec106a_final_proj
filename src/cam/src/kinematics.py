from __future__ import division
import numpy as np
import numpy.linalg as la
from math import sin, cos, sqrt, atan2
from geometry_msgs.msg import Pose, Point, Quaternion

VICON_TO_CRAZY = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

def skew_3d(omega):
    """
    Converts a rotation vector in 3D to its corresponding skew-symmetric matrix.
    
    Args:
    omega - (3,) ndarray: the rotation vector
    
    Returns:
    omega_hat - (3,3) ndarray: the corresponding skew symmetric matrix
    """
    omega_hat = np.array([[0, -omega[2], omega[1]], [omega[2], 0, -omega[0]], [-omega[1], omega[0], 0]])

    return omega_hat

def rotation_3d(omega, theta):
    """
    Computes a 3D rotation matrix given a rotation axis and angle of rotation.
    
    Args:
    omega - (3,) ndarray: the axis of rotation
    theta: the angle of rotation
    
    Returns:
    rot - (3,3) ndarray: the resulting rotation matrix
    """
    wHat = skew_3d(omega)
    wMag = la.norm(omega)
    rot = np.identity(3) + wHat * sin(wMag * theta) / wMag + la.matrix_power(wHat, 2) * (1 - cos(wMag * theta)) / wMag**2

    return rot

def rotation_3d(omega, theta):
    """
    Computes a 3D rotation matrix given a rotation axis and angle of rotation.
    
    Args:
    omega - (3,) ndarray: the axis of rotation
    theta: the angle of rotation
    
    Returns:
    rot - (3,3) ndarray: the resulting rotation matrix
    """
    wHat = skew_3d(omega)
    wMag = la.norm(omega)
    rot = np.identity(3) + wHat * sin(wMag * theta) / wMag + la.matrix_power(wHat, 2) * (1 - cos(wMag * theta)) / wMag**2

    return rot

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
    
    return omega, theta

def rot_to_quaternion(rot):
    w = sqrt(1 + rot[0][0] + rot[1][1] + rot[2][2]) / 2
    x = (rot[1][2] + rot[2][1]) / (4*w)
    y = (rot[0][2] + rot[0][2]) / (4*w)
    z = (rot[0][1] + rot[1][0]) / (4*w)
    return x, y, z, w

def get_yaw_from_rot(rot):
    return atan2(rot[1][0], rot[0][0])


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
    R = rotation_3d(omega, theta)
    g = np.array([[R[0][0], R[0][1], R[0][2], p[0]], [R[1][0], R[1][1], R[1][2], p[1]], [R[2][0], R[2][1], R[2][2], p[2]], [0, 0, 0, 1]])
    
    return g

def tf_to_rbt(tf):
    quat = np.array([tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])
    omega, theta = quaternion_to_exp(quat)
    if la.norm(omega) == 0:
        omega, theta = np.array([1, 0, 0]), 0

    trans = np.array([tf.translation.x, tf.translation.y, tf.translation.z])

    return create_rbt(omega, theta, trans)

def rbt_to_pose(rbt):
    yaw = get_yaw_from_rot(rbt[:3,range(3)])
    trans = rbt[:3,3]
    p = Point(trans[0], trans[1], trans[2])
    o = Quaternion(0, 0, yaw, 0)
    return Pose(p, o)

def get_yaw_from_quat(quat):
    return atan2(2*(quat.x*quat.w + quat.y*quat.z), (1 - 2*(quat.y**2 + quat.z**2)))