from abc import abstractmethod
import numpy as np
from scipy.integrate import solve_ivp

# runge-kutta 4th order
#EOM is equations of motion for specific earth model
#def RK45(x):
    #rn = []
    #for i in range(len(x)):
        #rn.append(x[i])
    #r1 = EOM(rn) 
    #for i in range(len(x)):
        #rn[i] = x[i]+0.5*dt*r1[i]
    #r2 = EOM(rn)
    #for i in range(len(x)):
        #rn[i] = x[i]+0.5*dt*r2[i]
    #r3 = EOM(rn)
    #for i in range(len(x)):
        #rn[i] = x[i]+dt*r3[i]
    #r4 = EOM(rn)
    #for i in range(len(x)):
        #rn[i] = x[i] + (dt/6.0)*(r1[i]+2.0*r2[i]+2.0*r3[i]+r4[i])
    #return rn

# class to implement the initial value problem func from scipy
class Integration():
    
    # default integration method of runge-kutta 45
    def _init_(self,time0,state0,method='RK45'):
        self._time = time0
        self._state_vector = state0
        self._state_vector_derivatives = np.zeros_like(state0)
        self._method = method

    @property
    def state_vector(self):
        return self._state_vector

    @property
    def state_vector_derivatives(self):
        return self._state_vector_derivatives
    
    @property
    def time(self):
        return self._time

    # default RK45 integration with time step dt
    def integrate(self,dt,t_eval=None,dense_output=True):
        state0 = self.state_vector
        t_in = self._time
        t_span = (t_in,t_in+dt)
        method = self._method

        sol = solve_ivp(self.func,t_span,state0,method=method,t_eval=t_eval,dense_output=dense_output)
        if sol.status == -1:
            raise RuntimeError(f"Integration did not converge. t = {t_in}")
        
        #update time to where integration ended
        self._time = sol.t[-1]
        
        # state vector is the last state of the integration
        self._state_vector = sol.y[:,-1]

        return self._state_vector

    #defines the integration function
    @abstractmethod
    def func(self,t,x):
        raise NotImplementedError

class VehicleIntegration(Integration):
    
    def _init_(self,time0,tot_state,method='RK45'):
        state0 = self._get_vehicle_state_vector(tot_state)
        self.tot_state = self._implement_tot_state(tot_state)
        super()._init_(time0,state0,method=method)
        self.update_sim = None

    @abstractmethod
    def _implement_tot_state(self,tot_state):
        raise NotImplementedError

    @abstractmethod
    def _get_vehicle_state_vector(self,tot_state):
        raise NotImplementedError

    @abstractmethod
    def steady_state_trim_func(self,tot_state,enviroment,aircrft,control):
        raise NotImplementedError