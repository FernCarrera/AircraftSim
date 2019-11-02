import numpy as np
from numpy import sin,cos,arctan2
from numpy import linalg as LA
from constants import PI


"""
Local NED to body transformation matrix
theta:pitch/elevation, phi:roll/bank, psi:heading/azimuth

RETURNS: Local2body transformation matrix
"""



def L2B(Local_H,theta,phi,psi):
    Lbh = np.array([
                    [cos(theta) * cos(psi),cos(theta) * sin(psi),- sin(theta)],
                    [sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi), sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi), sin(phi) * cos(theta)],
                    [cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi),cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi), cos(phi) * cos(theta)]
                    ])

    Local_body = Lbh.dot(Local_H)

    return Local_body

def B2L(body_coords,theta,phi,psi):

    Lhb = np.array([
                    [cos(theta) * cos(psi),sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi),cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)],
                    [cos(theta) * sin(psi),sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi),cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)],
                    [- sin(theta),sin(phi) * cos(theta),cos(phi) * cos(theta)]
                    ])

    hor_coords = Lhb.dot(body_coords)

    return hor_coords
"""
In order of application: heading:theta,attitude:phi, bank:psi
qw,qx,qy,qz
"""
def Euler2quat(euler):
    theta = euler[0]
    phi = euler[1]
    psi = euler[2]

    c1 = cos(theta/2)
    s1 = sin(theta/2)
    c2 = cos(phi/2)
    s2 = sin(phi/2)
    c3 = cos(psi/2)
    s3 = sin(psi/2)
    c1c2 = c1*c2
    s1s2 = s1*s2
    q0 = c1c2*c3 - s1s2*s3
    q1 = c1c2*s3 + s1s2*c3
    q2 = s1*c2*c3 + c1*s2*s3
    q3 = c1*s2*c3 - s1*c2*s3

    return np.array([q0,q1,q2,q3])
"""
qw,qx,qy,qz
q0,q1,q2,q3
"""
def Quat2euler(quat):
    # checking for gimbal lock
    q0 = quat[0]
    q1 = quat[1]
    q2 = quat[2]
    q3 = quat[3]
    check = q1*q2 + q3*q0
    # north-pole singularity
    if check > 0.499:
        theta = 2*arctan2(q1,q0)
        phi = PI/2
        psi = 0

    # south-pole singularityy
    if check < -0.499:
        theta = -2*arctan2(q1,q0)
        phi = -PI/2
        psi = 0
    
    x = q1**2
    y = q2**2
    z = q3**2
    theta = arctan2(2*q2*q0 - 2*q1*q3 , 1 - 2*y - 2*z)
    phi = np.arcsin(2*check)
    psi = arctan2(2*q1*q0 - 2*q2*q3 , 1 - 2*x - 2*z)

    return np.array([theta,phi,psi])

# Returns normalized quaternion
def normquat(w,x,y,z):
    norm = LA.norm(w,x,y,z)
    q_norm = (w+x+y+z)/norm
    return q_norm