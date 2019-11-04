from Vehicle_Physics.angular_velocity import BodyAngVel
from Vehicle_Physics.acceleration import BodyAccel
from Vehicle_Physics.angular_acceleration import BodyAngAccel
import numpy as np

"""
Stores Aircraft state
"""

class VehicleState():

    # default to None unless specified by user
    def __init__(self,position,attitude,Velocity,ang_V=None,accel=None,ang_accel=None):
        self.position = position
        self.attitude = attitude
        self.velocity = Velocity

        #default checks
        if ang_V is None:
            ang_V = BodyAngVel(0,0,0,attitude)
            
        if accel is None:
            accel = BodyAccel(0,0,0,attitude)
            
        if ang_accel is None:
            ang_accel = BodyAngAccel(0,0,0,attitude)
            
        self.ang_V = ang_V
        self.acceleration = accel
        self.ang_accel = ang_accel



