"""
UAV Control

@author: elie
"""

import math
import numpy as np
import quaternion as quat

# constants
#MASS = 0.226
MASS = 0.470
G = 9.81

# control gains
Kp_pos = np.array([[0.5,0.5,0.8]]).T
Kd_pos = np.array([[0.5,0.5,0.4]]).T
Kp_psi = 1

def outerloop(p,pdot,psi,ref):
    psi_ref = math.radians(ref[3])
    p_ref = ref[0:3]
    
    # position control
    F_no_g = - np.multiply(Kp_pos,(p-p_ref)) - np.multiply(Kd_pos,pdot)
    F = F_no_g - np.matrix([[0,0,MASS*G]]).T
    
    Tc = -F_no_g[2,0] # thrust (or vertical speed)
    
    Fxy_norm = np.linalg.norm(F[0:2,0])
    alpha = math.atan2(Fxy_norm,-F[2,0])
    qw = math.cos(alpha/2)
    if Fxy_norm == 0:
        qxyz = np.zeros(3)
    else:
        qxyz = (math.sin(alpha/2)/Fxy_norm)*np.array([F[1,0],-F[0,0],0])
    qalpha = np.concatenate([[qw], qxyz])
    qpsi = np.array([math.cos(psi/2),0,0,math.sin(psi/2)])
    qc = quat.multiply(qalpha,qpsi)
    
    _, thetac, phic = quat.q_to_euler(qc) # pitch, roll (rad)
    
    # yaw control (rad/s)
    delta_psi = psi-psi_ref
    if delta_psi < -math.pi:
        delta_psi += 2*math.pi
    elif delta_psi > math.pi:
        delta_psi -= 2*math.pi
    psic = - Kp_psi*delta_psi
    
    return (Tc,phic,thetac,psic)