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
    def __init__(self,time0,state0,method='RK45',options=None):
        """[Base Init of integration class]
        
        Arguments:
            time0 {[integer]} -- [Initial time]
            state0 {[array]} -- [Initial state of vehicle]
        
        Keyword Arguments:
            method {str} -- [integration method] (default: {'RK45'})
        """
        if options is None:
            options = {}
        self._time = time0
        self._state_vector = state0
        self._state_vector_derivatives = np.zeros_like(state0)
        
        self._method = method
        self._options = options
    @property
    def state_vector(self):
        return self._state_vector

    @property
    def state_vector_derivatives(self):
        return self._state_vector_derivatives
    
    @property
    def time(self):
        return self._time


    def integrate(self,t_end,t_eval=None,dense_output=True):

        x0 = self.state_vector
        t_ini = self.time

        t_span = (t_ini, t_end)
        method = self._method

        # TODO: prepare to use jacobian in case it is defined
        sol = solve_ivp(self.func, t_span, x0, method=method, t_eval=t_eval,dense_output=dense_output,**self._options)

        self._time = sol.t[-1]
        self._state_vector = sol.y[:, -1]

        return sol.t, sol.y, sol

    # default RK45 integration with time step dt
    def time_step(self,dt):
        x0 = self.state_vector
        t_ini = self.time

        t_span = (t_ini, t_ini + dt)
        method = self._method

        # TODO: prepare to use jacobian in case it is defined
        sol = solve_ivp(self.func_wrap, t_span, x0, method=method,
                        **self._options)

        if sol.status == -1:
            raise RuntimeError(f"Integration did not converge at t={t_ini}")

        self._time = sol.t[-1]
        self._state_vector = sol.y[:, -1]

        return self._state_vector

    #defines the integration function
    @abstractmethod
    def func(self,t,x):
        raise NotImplementedError
    #stores derivatives for calculating total state
    def func_wrap(self,t,x):
        state_derivatives = self.func(t,x)
        self._state_vector_derivatives = state_derivatives
        return state_derivatives

class VehicleIntegration(Integration):
    
    def __init__(self,time0,tot_state,method='RK45',options=None):
        state0 = self._get_vehicle_state_vector(tot_state)
        self.tot_state = self._adapt_tot_state(tot_state)
        super().__init__(time0,state0,method=method,options=options)
        self.update_simulation = None

    @abstractmethod
    def _adapt_tot_state(self,tot_state):
        raise NotImplementedError

    @abstractmethod
    def _update_tot_state(self,state,state_derivatives):
        raise NotImplementedError


    @abstractmethod
    def _get_vehicle_state_vector(self,tot_state):
        raise NotImplementedError

    @abstractmethod
    def steady_state_trim_func(self,tot_state,enviroment,aircrft,control):
        raise NotImplementedError

    def time_step(self,dt):
        super().time_step(dt)
        self._update_tot_state(self.state_vector,self.state_vector_derivatives)

        return self.tot_state