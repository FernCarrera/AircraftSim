from abc import abstractmethod
import numpy as np

"""
angular velocity p,q,r [rad/s]
&
euler angular rates theta_dot,phi_dot,psi_dot [rad/s]
"""

class AngularVelocity():

    def _init_(self):
        self._ang_v_body = np.zeros(3)
        self._ang_rates = np.zeros(3)

    @abstractmethod
    def update(self,value,attitude):
        raise NotImplementedError

    @property
    def ang_v_body(self):
        return self._ang_v_body

    @property
    def p(self):
        return self._ang_v_body[0]

    @property
    def q(self):
        return self._ang_v_body[1]

    @property
    def r(self):
        return self._ang_v_body[2]

    @property
    def ang_rates(self):
        return self._ang_rates

    @property
    def theta_dot(self):
        return self._ang_rates[0]    

    @property
    def phi_dot(self):
        return self._ang_rates[1]    

    @property
    def psi_dot(self):
        return self._ang_rates[2]    
    
class BodyAngVel(AngularVelocity):

    def _init_(self,p,q,r,attitude):
        super()._init_()
        self.update(np.array([p,q,r]),attitude)

    def update(self,value,attitude):
        self.ang_v_body[:] = value
        # need to convert
        self._ang_rates = np.zeros(3)

class EulerAngRates(AngularVelocity):

    def _init_(self,t_dot,ph_dot,ps_dot,attitude):
        super()._init_()
        self.update(np.array([t_dot,ph_dot,ps_dot]),attitude)

    def update(self,value,attitude):
        self.ang_v_body = np.zeros(3)
        # need to convert
        self._ang_rates[:] = value
