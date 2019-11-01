import numpy as np
from numpy import sin,cos

def hor2body(Local_H,theta,phi,psi):
    Lbh = np.array([
                    [cos(theta) * cos(psi),cos(theta) * sin(psi),- sin(theta)],
                    [sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi), sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi), sin(phi) * cos(theta)],
                    [cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi),cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi), cos(phi) * cos(theta)]
                    ])

    Local_body = Lbh.dot(Local_H)

    return Local_body