import numpy as np
from numpy import sin,cos,arctan2
from numpy import linalg as LA
from Utility.constants import PI,EARTH_RADIUS


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

    ct = cos(theta/2)
    cph = cos(phi/2)
    cps = cos(psi/2)

    st = sin(theta/2)
    sph = sin(phi/2)
    sps = sin(psi/2)

    
    q0 = cps*ct*cph + sps*st*sph
    q1 = cps*ct*sph - sps*st*cph
    q2 = cps*st*cph + sps*ct*sph
    q3 = sps*ct*cph - cps*st*sph

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


def check_alpha_beta_range(alpha, beta):
    """Check alpha, beta values are inside the defined range. This
    comprobation can also detect if the value of the angle is in degrees in
    some cases.
    """

    alpha_min, alpha_max = (-np.pi/2, np.pi/2)
    beta_min, beta_max = (-np.pi, np.pi)

    if not (alpha_min <= alpha <= alpha_max):
        raise ValueError('Alpha value is not inside correct range')
    elif not (beta_min <= beta <= beta_max):
        raise ValueError('Beta value is not inside correct range')

def wind2body(wind_coords,alpha,beta):

    check_alpha_beta_range(alpha, beta)

    # Transformation matrix from body to wind
    Lbw = np.array([
                    [cos(alpha) * cos(beta),- cos(alpha) * sin(beta),-sin(alpha)],
                    [sin(beta),cos(beta),0],
                    [sin(alpha) * cos(beta),-sin(alpha) * sin(beta),cos(alpha)]
                    ])

    body_coords = Lbw.dot(wind_coords)

    return body_coords

def geopotential2geometric(height):
    """[converts geopotential heightitude to geometric ]
    
    Arguments:
        height {[float]} -- [geopotential heightitude]
    """

    return EARTH_RADIUS*height/(EARTH_RADIUS-height)

def geometric2geopotential(height):
    """[converts geometric to geopotential heightitutde]
    
    Arguments:
        height {[float]} -- [geometric heightitude]
    """
    return EARTH_RADIUS * height / (EARTH_RADIUS + height)