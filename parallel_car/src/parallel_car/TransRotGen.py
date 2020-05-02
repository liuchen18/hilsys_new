#!/usr/bin/env python 

import rospy

import numpy as np

import math



def Rotation(axis, angle, in_degree=False):
    """A funtion to return homegeneous rotation matrix
    
    Arguments:
        axis {string} -- either 'x' or 'y' or 'z'
        angle {float} -- in degrees
        in_degree {bool} -- if True do conversion from degree to radian
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous rotation matrix
    """
    if in_degree:
        angle = np.deg2rad(angle)
    else:
        pass
    if axis in ['x', 'y', 'z']:
        if axis == 'x':
            return Rot_X(angle)
        elif axis == 'y':
            return Rot_Y(angle)
        else: # axis == 'z'
            return Rot_Z(angle)
    else:
        rospy.logerr('Axis wrong! Return identity matrix')
        return np.mat(np.eye(4,4))

def Rot_X(alpha):
    """A function to return homogeneous rotation matrix along x-axis
    
    Arguments:
        alpha {float} -- angle in radians
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous rotation matrix along x-axis
    """
    T_rot_X = np.mat(np.eye(4,4))

    T_rot_X[1, 1] = np.cos(alpha)
    T_rot_X[2, 2] = np.cos(alpha)
    T_rot_X[1, 2] = -np.sin(alpha)
    T_rot_X[2, 1] = np.sin(alpha)
    
    return T_rot_X

def Rot_Y(beta):
    """A function to return homogeneous rotation matrix along y-axis
    
    Arguments:
        beta {float} -- angle in radians
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous rotation matrix along y-axis
    """
    T_rot_Y = np.mat(np.eye(4,4))

    T_rot_Y[0, 0] = np.cos(beta)
    T_rot_Y[2, 2] = np.cos(beta)
    T_rot_Y[0, 2] = np.sin(beta)
    T_rot_Y[2, 0] = -np.sin(beta)
    
    return T_rot_Y

def Rot_Z(gamma):
    """A function to return homogeneous rotation matrix along z-axis
    
    Arguments:
        gamma {float} -- angle in radians
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous rotation matrix along y-axis
    """
    T_rot_Z = np.mat(np.eye(4,4))

    T_rot_Z[0, 0] = np.cos(gamma)
    T_rot_Z[1, 1] = np.cos(gamma)
    T_rot_Z[0, 1] = -np.sin(gamma)
    T_rot_Z[1, 0] = np.sin(gamma)

    return T_rot_Z

def Translation(axis, distance):
    """A funtion to return homegeneous translation matrix
    
    Arguments:
        axis {string} -- either 'x' or 'y' or 'z'
        distance {float} -- the distance to travel along the axis
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous translation matrix
    """
    if axis in ['x', 'y', 'z']:
        if axis == 'x':
            return Trans_X(distance)
        elif axis == 'y':
            return Trans_Y(distance)
        else: # axis == 'z'
            return Trans_Z(distance)
    else:
        rospy.logerr('Axis wrong! Return identity matrix')
        return np.mat(np.eye(4,4))

def Trans_X(dist):
    """A funtion to return homogeneous translation matrix along x-axis
    
    Arguments:
        dist {float} -- distance to travel along x-axis
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous translation matrix along x-axis
    """
    T_trans_X = np.mat(np.eye(4,4))

    T_trans_X[0, 3] = dist

    return T_trans_X

def Trans_Y(dist):
    """A funtion to return homogeneous translation matrix along y-axis
    
    Arguments:
        dist {float} -- distance to travel along y-axis
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous translation matrix along y-axis
    """
    T_trans_Y = np.mat(np.eye(4,4))

    T_trans_Y[1, 3] = dist

    return T_trans_Y

def Trans_Z(dist):
    """A funtion to return homogeneous translation matrix along z-axis
    
    Arguments:
        dist {float} -- distance to travel along z-axis
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous translation matrix along z-axis
    """
    T_trans_Z = np.mat(np.eye(4,4))

    T_trans_Z[2, 3] = dist

    return T_trans_Z

def transform_to_matrix(vec3, quat):
    """A function to get homogeneous matrix from transform
    
    Arguments:
        vec3 {Vector3} -- Vector3-type msg from given transform
        quat {Quaternion} -- Quaternion-type msg from given transform
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous matrix representing given transfrom
    """
    trans_matrix = vector3_to_translation_matrix(vec3)
    rot_matrix = quaternion_to_rotation_matrix(quat)

    # because of the transformation from tf, multiply trans_matrix first
    com_matrix = trans_matrix*rot_matrix
    
    return com_matrix


def quaternion_to_rotation_matrix(quat):
    """A function to transform quaternion to homogeneous rotation matrix
    
    Arguments:
        rot {Quaternion} -- Quaternion-type msg
    
    Returns:
        Numpy matrix in 4x4  -- Result homogeneous rotation matrix from input quaternion
    """
    # get qx, qy, qz and qw from Quaternion-type msg
    qx = quat.x
    qy = quat.y
    qz = quat.z
    qw = quat.w

    rot_matrix = np.mat(np.eye(4))

    rot_matrix[0, 0] = 1-2*np.square(qy)-2*np.square(qz)
    rot_matrix[0, 1] = 2*qx*qy-2*qz*qw
    rot_matrix[0, 2] = 2*qx*qz+2*qy*qw
    rot_matrix[1, 0] = 2*qx*qy+2*qz*qw
    rot_matrix[1, 1] = 1-2*np.square(qx)-2*np.square(qz)
    rot_matrix[1, 2] = 2*qy*qz-2*qx*qw
    rot_matrix[2, 0] = 2*qx*qz-2*qy*qw
    rot_matrix[2, 1] = 2*qy*qz+2*qx*qw
    rot_matrix[2, 2] = 1-2*np.square(qx)-2*np.square(qy)
    
    return rot_matrix


def vector3_to_translation_matrix(vec3):
    """A function to transfrom from Vector3-type msg to homogeneous translation matrix
    
    Arguments:
        vec3 {Vector3} -- Vector3-type msg
    
    Returns:
        Numpy matrix in 4x4  -- Result homogeneous translation matrix from input vector3
    """
    
    # get x, y and z from Vector3-type msg
    x = vec3.x
    y = vec3.y
    z = vec3.z

    trans_matrix = np.mat(np.eye(4))
    trans_matrix[0, 3] = x
    trans_matrix[1, 3] = y
    trans_matrix[2, 3] = z

    return trans_matrix

def quaternion_to_euler(quat):
    (x, y, z, w) = (quat.x, quat.y, quat.z, quat.w)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

def yaw_from_quaternion_only_Z(quat):
    """A function to get yaw from quaternion which ONLY contains rotation about z-axis

    Arguments:
        quat {Quaternion} -- a quaternion resulted from rotation only about z-axis

    Returns:
        [double] -- yaw or rotation angle about z-axis
    """
    T_rot_Z = quaternion_to_rotation_matrix(quat)

    yaw = np.arctan2(T_rot_Z[1, 0], T_rot_Z[0, 0])

    return yaw

def rpy_from_quaternion(quat):
    """A function to get roll, pitch, yaw respectively from quaternion

    Arguments:
        quat {Quaternion} -- a quaternion resulted from rotation first about x-axis, then about y-axis and finally about z-axis

    Returns:
        [tuple] -- a tuple containing roll, pitch, yaw respectively
    """

    T_rot = quaternion_to_rotation_matrix(quat)

    # alpha, beta, gamma are all restricted to (-pi/2, pi/2), so sin(beta) can be used to calculate beta
    # alpha: roll : rotation about x-axis
    # beta : pitch: rotation about y-axis
    # gamma: yaw  : rotation about z-axis

    alpha = np.arctan2(-T_rot[1, 2], T_rot[2, 2])
    
    beta = np.arcsin(T_rot[0, 2])
    
    gamma = np.arctan2(-T_rot[0, 1], T_rot[0, 0])

    return (alpha, beta, gamma)