from abc import abstractmethod
import numpy as np


"""
angular acceleration pdot,qdot,pdot [rad/s**2]
euler angular accel theta_2dot,phi_2dot,psi_2dot [rad/s**2]
"""
class AngularAcceleration():


    def __init__(self):
        self._ang_acc_body = np.zeros(3)
        self._ang_acc_euler = np.zeros(3)

    @abstractmethod
    def update(self,value,attitude):
        return NotImplementedError

    @property
    def ang_acc_body(self):
        return self._ang_acc_body

    @property
    def p_dot(self):
        return self._ang_acc_body[0]

    @property
    def q_dot(self):
        return self._ang_acc_body[1]

    @property
    def r_dot(self):
        return self._ang_acc_body[2]

    @property
    def ang_acc_euler(self):
        return self._ang_acc_euler

    @property
    def theta_2dot(self):
        return self._ang_acc_euler[0]

    @property
    def phi_2dot(self):
        return self._ang_acc_euler[1]

    @property
    def psi_2dot(self):
        return self._ang_acc_euler[2]

class BodyAngAccel(AngularAcceleration):

    def __init__(self,p_dot,q_dot,r_dot,attitude):
        super().__init__()
        self.update(np.array([p_dot,q_dot,r_dot]),attitude)

    def update(self,value,attitude):
        self._ang_acc_body[:] = value
        """" TODO: can get local accelerations from body accel using an
                    integrated L2B matrix, will implement soon
        """
        self._ang_acc_euler = np.zeros(3)

class EulerAngAccel(AngularAcceleration):

    def __init__(self,theta_2dot,phi_2dot,psi_2dot,attitude):
        super().__init__()
        self.update(np.array([theta_2dot,phi_2dot,psi_2dot]),attitude)

    def update(self,value,attitude):
        self._ang_acc_euler[:] = value
        """" TODO: can get body accelerations from local accel using an
                    integrated L2B matrix, will implement soon
        """
        self._ang_acc_body = np.zeros(3)

