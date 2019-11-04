from abc import abstractmethod
import numpy as np
from Utility.Transformations import L2B, B2L
"""
Acceleration in the body frame and Local horizon NED
"""
class Acceleration():

    def __init__(self):
        self._accel_body = np.zeros(3)
        self._accel_NED = np.zeros(3)

    @abstractmethod
    def update(self,accels,attitude):
        raise NotImplementedError
    """body frame"""
    @property
    def accel_body(self):
        return self._accel_body
    @property
    def u_dot(self):
        return self._accel_body[0]
    @property
    def v_dot(self):
        return self._accel_body[1]
    @property
    def w_dot(self):
        return self._accel_body[2]
    """NED frame"""
    @property
    def accel_NED(self):
        return self._accel_NED
    
    @property
    def vn_dot(self):
        return self._accel_NED[0]

    @property
    def ve_dot(self):
        return self._accel_NED[1]

    @property
    def vd_dot(self):
        return self._accel_NED[2]

class BodyAccel(Acceleration):

    def __init__(self,u_dot,v_dot,w_dot,attitude):
        super().__init__()
        self.update(np.array([u_dot,v_dot,w_dot]),attitude)
    
    def update(self,accels,attitude):
        self._accel_body[:] = accels
        self._accel_NED = B2L(accels,attitude.theta,attitude.phi,attitude.psi)

class NEDAccel(Acceleration):

    def __init__(self,vn_dot,ve_dot,vd_dot,attitude):
        super().__init__()
        self.update(np.array([vn_dot,ve_dot,ve_dot]),attitude)

    def update(self,accels,attitude):
        self._accel_NED[:] = accels
        self._accel_body = L2B(accels,attitude.theta,attitude.phi,attitude.psi)