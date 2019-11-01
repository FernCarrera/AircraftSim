from abc import abstractmethod
from Utility.Transformations import body2hor, hor2body
import numpy as np

class Velocity():

    def __init__(self):
        self._v_body = np.zeros(3)      # m/s
        self._v_NED = np.zeros(3)       # m/s

    @abstractmethod
    def update(self,vels,attitude):
        raise NotImplementedError
    """body velocity"""
    @property
    def v_body(self):
        return self._v_body

    @property
    def u(self):
        return self.v_body[0]

    @property
    def v(self):
        return self.v_body[1]
    
    @property
    def w(self):
        return self.v_body[2]
    """NED velocity"""
    @property
    def v_north(self):
        return self._v_NED[0]

    @property
    def v_east(self):
        return self._v_NED[1]
    
    @property
    def v_down(self):
        return self._v_NED[2]

"""
Updating velocity via body velocity update
"""
class BodyVelocity(Velocity):

    def __init__(self,u,v,w,attitude):
        super().__init__()
        self.update(np.array([u, v, w]), attitude)

    def update(self,value,attitude):
        self._v_body[:] = value
        self._v_NED = body2hor(value,attitude.theta,attitude.phi,attitude.psi)

"""
Updating velocity via NED velocity update
"""
class NEDVelocity(Velocity):

    def __init__(self,vn,ve,vd,attitude):
        super().__init__()
        self.update(np.array([vn,ve,vd]), attitude)

    def update(self,value,attitude):
        self._v_NED[:] = value
        self._v_body = hor2body(value,attitude.theta,attitude.phi,attitude.psi)

