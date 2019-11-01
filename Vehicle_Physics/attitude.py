from abc import abstractmethod
from Utility.Transformations import Euler2quat,Quat2euler
import numpy as np


"""
Holds Euler[rad] and Quaternion attitudes
"""
class Attitude():


    def __init__(self):
        self._euler_angles = np.zeros(3)
        self._quaternions = np.zeros(4)

    @abstractmethod
    def update(self,value):
        raise NotImplementedError

    @property
    def euler_angles(self):
        return self._euler_angles

    @property
    def psi(self):
        return self._euler_angles[0]

    @property
    def theta(self):
        return self._euler_angles[1]

    @property
    def phi(self):
        return self._euler_angles[2]

    @property
    def quaternions(self):
        return self._quaternions

    @property
    def q0(self):
        return self._quaternions[0]

    @property
    def q1(self):
        return self._quaternions[1]

    @property
    def q2(self):
        return self._quaternions[2]

    @property
    def q3(self):
        return self._quaternions[3]

class EulerAttitude(Attitude):

    def __init__(self,theta,phi,psi):
        super().__init__()
        self.update(np.array([theta,phi,psi]))

    def update(self,value):
        self._euler_angles[:] = value

        self._quaternions = Euler2quat(self.euler_angles)

class QuaternionAttitude(Attitude):

    def __init__(self,q0,q1,q2,q3):
        super().__init__()
        self.update(np.array([q0,q1,q2,q3]))

    def update(self,value):
        self._euler_angles = Quat2euler(self.quaternions)
        self._quaternions[:] = value