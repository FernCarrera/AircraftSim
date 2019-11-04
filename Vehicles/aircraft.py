from abc import abstractmethod
import numpy as np
from Utility.anemometry import TAS2CAS,TAS2EAS,calc_alpha_beta_TAS 

"""
generic base class for aircraft
"""

class Aircraft():


    def __init__(self):
        
        # Mass and Inertia Properties
        self.mass = 0                           #kg
        self.inertia = np.zeros((3,3))          # k*m**2

        # Vehicle Geometry
        self.S = 0                              # Wing area m**2
        self.chord = 0                          # chord length m
        self.span = 0                           # wing span m

        # Control
        self.controls = {}
        self.control_limits = {}

        # Aerodynamic Coefficients
        self.CL, self.CD, self.Cm = 0, 0, 0
        self.CY, self.Cl, self.Cn = 0, 0, 0

        # Thrust Coefficient
        self.Ct = 0

        # Force and Moment Vectors
        self.forces = np.zeros(3)
        self.moments = np.zeros(3)

        # Velocities
        self.TAS = 0                            # true airspeed
        self.CAS = 0                            # calibrated airspeed
        self.EAS = 0                            # equivalent airspeed
        self.Mach = 0                           # mach number
        self.q_inf = 0                          # dynamic pressure @ infinity [Pa]

        # Angles
        self.alpha = 0                          # angle of attack, AOA
        self.beta = 0                           # angle of sideslip, AOS
        self.alpha_dot = 0                      # rate of change of AOA

    # getters
    @property
    def Ixx(self):
        return self.inertia[0,0]

    @property
    def Iyy(self):
        return self.inertia[1,1]
    
    @property
    def Izz(self):
        return self.inertia[2,2]

    @property
    def Fx(self):
        return self.forces[0]

    @property
    def Fy(self):
        return self.forces[1]

    @property
    def Fz(self):
        return self.forces[2]

    @property
    def Mx(self):
        return self.moments[0]

    @property
    def My(self):
        return self.moments[1]

    @property
    def Mz(self):
        return self.moments[2]

    #privates
    #sets controls and control limits to the control surfaces
    def _set_controls(self,controls):
        for control_name, control_value in controls.items():
            limits = self.control_limits[control_name]
            if limits[0] <= control_value <= limits[1]:
                self.controls[control_name] = control_value
            else:
                # TODO:  raise a warning & assign max deflection
                msg = (
                    f"Control {control_name} out of range ({control_value} "
                    f"when min={limits[0]} and max={limits[1]})"
                )
                raise ValueError(msg)

    # calculates velocities and dyn pressure
    def _calc_aerodynamics(self,state,environment):
        aero_v = state.velocity.v_body - environment.body_wind
        self.alpha,self.beta,self.TAS = calc_alpha_beta_TAS(u=aero_v[0],v=aero_v[1],w=aero_v[2])
        
        # Velocity and Aerodynamic pressue

        self.CAS = TAS2CAS(self.TAS,environment.pressure,environment.rho)
        self.EAS = TAS2EAS(self.TAS,environment.rho)
        self.Mach = self.TAS/environment.sos
        self.q_inf = 0.5*environment.rho*self.TAS**2

    def _calc_aerodynamics_2(self, TAS, alpha, beta, environment):

        self.alpha, self.beta, self.TAS = alpha, beta, TAS

        # Setting velocities & dynamic pressure
        self.CAS = TAS2CAS(self.TAS, environment.pressure, environment.rho)
        self.EAS = TAS2EAS(self.TAS, environment.rho)
        self.Mach = self.TAS / environment.sos
        self.q_inf = 0.5 * environment.rho * self.TAS ** 2

    @abstractmethod
    def calc_forces_and_moments(self,state,environment,controls):
        self._set_controls(controls)
        self._calc_aerodynamics(state,environment)