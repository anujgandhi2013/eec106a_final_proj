import numpy as np
from math import sin, cos
from numpy.linalg import norm, matrix_power

def skew_3d(omega):
    """
    Converts a rotation vector in 3D to its corresponding skew-symmetric matrix.
    
    Args:
    omega - (3,) ndarray: the rotation vector
    
    Returns:
    omega_hat - (3,3) ndarray: the corresponding skew symmetric matrix
    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    
    omega_hat = np.array([[0, -omega[2], omega[1]], [omega[2], 0, -omega[0]], [-omega[1], omega[0], 0]])

    return omega_hat

def rotation_2d(theta):
    """
    Computes a 2D rotation matrix given the angle of rotation.
    
    Args:
    theta: the angle of rotation
    
    Returns:
    rot - (3,3) ndarray: the resulting rotation matrix
    """
    
    rot = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])

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
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    
    wHat = skew_3d(omega)
    wMag = norm(omega)
    rot = np.identity(3) + wHat * sin(wMag * theta) / wMag + matrix_power(wHat, 2) * (1 - cos(wMag * theta)) / wMag**2

    return rot

def hat_2d(xi):
    """
    Converts a 2D twist to its corresponding 3x3 matrix representation
    
    Args:
    xi - (3,) ndarray: the 2D twist
    
    Returns:
    xi_hat - (3,3) ndarray: the resulting 3x3 matrix
    """
    if not xi.shape == (3,):
        raise TypeError('omega must be a 3-vector')

    xi_hat = np.array([[0, -xi[2], xi[0]], [xi[2], 0, xi[1]], [0, 0, 0]])

    return xi_hat

def hat_3d(xi):
    """
    Converts a 3D twist to its corresponding 4x4 matrix representation
    
    Args:
    xi - (6,) ndarray: the 3D twist
    
    Returns:
    xi_hat - (4,4) ndarray: the corresponding 4x4 matrix
    """
    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')

    xi_hat = np.array([[0, -xi[5], xi[4], xi[0]], [xi[5], 0, -xi[3], xi[1]], [-xi[4], xi[3], 0, xi[2]], [0, 0, 0, 0]])

    return xi_hat

def homog_2d(xi, theta):
    """
    Computes a 3x3 homogeneous transformation matrix given a 2D twist and a 
    joint displacement
    
    Args:
    xi - (3,) ndarray: the 2D twist
    theta: the joint displacement
    
    Returns:
    g - (3,3) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (3,):
        raise TypeError('xi must be a 3-vector')

    R = rotation_2d(xi[2] * theta)
    p = np.array([[1 - cos(xi[2] * theta), sin(xi[2] * theta)], [-sin(xi[2] * theta), 1 - cos(xi[2] * theta)]])
    p = p.dot(np.array([[0, -1], [1, 0]]))
    p = p.dot(np.array([[xi[0] / xi[2]], [xi[1] / xi[2]]]))
    g = np.array([[R[0][0], R[0][1], p[0][0]], [R[1][0], R[1][1], p[1][0]], [0, 0, 1]])

    return g

def homog_3d(xi, theta):
    """
    Computes a 4x4 homogeneous transformation matrix given a 3D twist and a 
    joint displacement.
    
    Args:
    xi - (6,) ndarray: the 3D twist
    theta: the joint displacement

    Returns:
    g - (4,4) ndarary: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')

    w = np.array([[xi[3], xi[4], xi[5]]])
    v = np.array([[xi[0], xi[1], xi[2]]])
    
    E = rotation_3d(xi[3:], theta)
    
    p11 = (np.identity(3) - E)
    p12 = np.dot(skew_3d(xi[3:]), v.T)
    p1 = np.dot(p11, p12)
    
    p2 = np.dot(w, v.T)
    p2 = p2 * w.T * theta
    
    p = p1 + p2
    p = p / norm(w)**2
    
    g = np.array([[E[0][0], E[0][1], E[0][2], p[0]], [E[1][0], E[1][1], E[1][2], p[1]], [E[2][0], E[2][1], E[2][2], p[2]], [0, 0, 0, 1]])

    return g

def prod_exp(xi, theta):
    """
    Computes the product of exponentials for a kinematic chain, given 
    the twists and displacements for each joint.
    
    Args:
    xi - (6,N) ndarray: the twists for each joint
    theta - (N,) ndarray: the displacement of each joint
    
    Returns:
    g - (4,4) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape[0] == 6:
        raise TypeError('xi must be a 6xN')

    n = 1
    xi = xi.T
    g = homog_3d(xi[0], theta[0])
    while n != theta.size:
        g = g.dot(homog_3d(xi[n], theta[n]))
        n += 1

    return g
    