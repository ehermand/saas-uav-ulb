"""
Unit Quaternion Tools

@author: elie
"""

import numpy as np
from math import atan2,asin,pi
import math

IDENTITY = np.array([1,0,0,0])

def multiply(quaternion1,quaternion0):
    """ q = q1*q0 """
    qw0, qx0, qy0, qz0 = quaternion0
    qw1, qx1, qy1, qz1 = quaternion1
    return np.array([qw1*qw0 -qx1*qx0 - qy1*qy0 - qz1*qz0,
                     qx1*qw0 + qy1*qz0 - qz1*qy0 + qw1*qx0,
                     -qx1*qz0 + qy1*qw0 + qz1*qx0 + qw1*qy0,
                     qx1*qy0 - qy1*qx0 + qz1*qw0 + qw1*qz0])

def normalize(quaternion):
    return quaternion/np.linalg.norm(quaternion)
    
def conjugate(quaternion):
    qw, qx, qy, qz = quaternion
    return np.array([qw, -qx, -qy, -qz])

def to_rotation_matrix(quaternion):
    qw, qx, qy, qz = quaternion
    return np.array([[(1-2*(qy**2)-2*(qz**2)),2*(qx*qy+qw*qz),2*(qx*qz-qw*qy)],
                     [2*(qx*qy-qw*qz),(1-2*(qx**2)-2*(qz**2)),2*(qy*qz+qw*qx)],
                     [2*(qx*qz+qw*qy),2*(qy*qz-qw*qx),(1-2*(qx**2)-2*(qy**2))]])
    
def q_to_euler(quaternion, order = 'wxyz'):
    if order == 'xyzw':
        qx, qy, qz, qw = quaternion
    else:
        qw, qx, qy, qz = quaternion
    
    test = qw*qy - qz*qx
    if (test > 0.499): # singularity at north pole
        bank = atan2(qx,qw)
        attitude = pi/2
        heading = 0
    elif (test < -0.499): # singularity at south pole
        bank = atan2(qx,qw)
        attitude = -pi/2
        heading = 0
    else:
        bank = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))
        attitude = asin(2*(qw*qy - qz*qx))
        heading = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
    return (heading, attitude, bank)    

def q_between_vectors(vector0,vector1):
    #TODO: add handling of opposite vectors (180° rotation)
    dot = np.dot(vector0,vector1)
    s = np.sqrt((1+dot)*2)
    invs = 1/s
    cross = np.cross(vector0,vector1)*invs
    return normalize(np.concatenate(([s*0.5],cross)))

def reorder(quaternion, dest='wxyz'):
    if dest == 'wxyz':
        shift = 1
    elif dest == 'xyzw':
        shift = -1
    else:
        shift = 0
        print("Invalid destination order. Quaternion untouched.")
    return np.roll(quaternion,shift)