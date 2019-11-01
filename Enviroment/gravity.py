from abc import abstractmethod
from Utility.Transformations import hor2body
import numpy as np
from Utility.constants import GRAVITY, STD_GRAVITATIONAL_PARAM
"""
Constant and non-constant gravity models
"""

class Gravity():

    def __init__(self):
        self._magnitude = None
        self._versor = np.zeros([3])
        self._vector = np.zeros([3])

    @property
    def magnitude(self):
        return self._magnitude

    @property
    def versor(self):
        return self._versor

    @property
    def vector(self):
        return self._vector

    @abstractmethod
    def update(self,state):
        pass

# standard gravitational model
class VerticalConstant(Gravity):
    def __init__(self):
        self._magnitude = GRAVITY
        self._z_horizon = np.array([0.0, 0.0, 1.0])

    
    def update(self,state):
        # converts from local horizon to local body
        self._versor = hor2body(self._z_horizon,
                                theta=state.attitude.theta,
                                phi=state.attitude.phi,
                                psi=state.attitude.psi
                                )
        self._vector = self.magnitude * self.versor