#!/usr/bin/env python
# coding=latin1

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

"""
Recebe um valor e um limite inferior e superior e retorna o próprio valor
ou um dos limites, caso o valor não esteja entre eles.
O limite é inclusivo no mínimo e exclusivo no máximo.
"""
def bound(v, min, max):
    if v < min:
        return min
    elif v > max-1:
        return max-1
    else:
        return v

"""
Recebe um valor e um limite inferior e superior e retorna o próprio valor
ou um dos limites, caso o valor não esteja entre eles.
O limite é inclusivo no mínimo e exclusivo no máximo.
"""
def quaternion_to_theta (x, y, z, w):
    # conversão de quatérnio para rollPitchYaw
    quaternion = (x,y,z,w)
    rollPitchYaw = euler_from_quaternion(quaternion)
    return rollPitchYaw[2] # Yaw

"""
Recebe um ângulo em graus positivo ou negativo e retorna o ângulo
positivo equivalente
"""
def modulo360 (angulo):
    base = 360
    return ((angulo % base) + base) % base


class Pose:
    def __init__(self, x=None, y=None, theta=None, odom_msg=None):
        if odom_msg is None:
            self.x = x
            self.y = y
            self.theta = theta
        else:
            pose = odom_msg.pose.pose
            self.x = pose.position.x
            self.y = pose.position.y
            self.theta = quaternion_to_theta(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

    def get_xy_array(self):
        return np.array([self.x, self.y])


class Point:
    def __init__(self, x=None, y=None, array=None):
        if array is not None:
            self.x = array[0]
            self.y = array[1]
        else:
            self.x = x
            self.y = y
    def __str__(self):
        return "(%.2f, %.2f)" % (self.x, self.y)
    def to_array(self):
        return np.array([self.x, self.y])
