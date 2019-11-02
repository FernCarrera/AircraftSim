"""
3dof earth models:
non-rotating spherical earth, rotating spherical earth
flat earth
"""
from Utility.Integration import Integration
from Vehicle_Physics._init_ import EarthPosition,EulerAngAccel,EulerAngRates,EulerAttitude,BodyAccel,BodyAngVel,BodyVelocity,VehicleState
import numpy as np
from numpy import sin,cos