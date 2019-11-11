import numpy as np
from Vehicles.aircraft import Aircraft


class Skyhawk(Aircraft):

    def __init__(self):
        self.mass = 7900                                # kg
        self.W = self.mass*9.81
        self.inertia = np.diag([10970,35100,39600])     # kg*m**2
        # Ixz = 1760

        # geometry
        self.S = 24.16                                  # m**2
        self.chord = 4.72                               # m
        self.span = 8.38                                # m
        self.c_mean = self.S/self.span                  # m



        """
        Aerodynamic model is a simplified model for a symmetric airplane
        in subsonic flight
        ----------------------------------------------------------------
        
        Long matrix
                                [1]
        [cl0 cla clq cla clde]  [a]     # alpha    
        [cd0 cda  0   0    0 ]  [qc/2v] # q: pitch, c: STD mean chord, v: airspeed
        [cm0 cma cmq cma cmde]  [*ac/2v]# *a: AOA rate
                                [de]    # elevator deflection
        """
        self._long_matrix = np.array([[0.0, 3.5, 0.0, 3.5, 0.4],
                                     [0.024, 1.3, 0.0 ,0.0, 0.0],
                                     [0.0,-0.4, 0.0, 0.0, -0.5 ]])

        """
        Lat Matrix              
                                [b]     # beta, side slip
        [cbl clp clr clda cldr] [pb/2v] # p: roll, b: wing span, v: airspeed
        [cyb cyp cyr cyda cydr] [rb/2v] # r: yaw
        [cnb cnp cnr cnda cndr] [da]    # da: aileron deflection
                                [dr]    # dr: rudder defelction
        """

        self._lat_matrix = np.array([[-1.0, 0.0 ,0.0 ,0.0 ,0.17],
                                    [-0.13, -0.25, 0.16, 0.0, 0.04],
                                    [0.26, 0.025, -0.37, 0.0, -0.11 ]])

        # input vector for longitude matrix
        self._long_in = np.zeros(5)
        self._long_in[0] = 1 # assign first value

        # input vector for latitude matrix
        self._lat_in = np.zeros(5)

        # Control Surface definition [rad]
        self.controls = {   'delta_elevator': 0.0,
                            'delta_aileron' : 0.0,
                            'delta_rudder'  : 0.0,
                            'delta_t'       : 0.0}  # change in thrust

        # need to research actual limits of control surfaces
        # using arbitrary control limits for cessna
        self.control_limits = {'delta_elevator': (np.deg2rad(-60),
                                                  np.deg2rad(60)),  # rad
                               'delta_aileron': (np.deg2rad(-60),
                                                 np.deg2rad(60)),  # rad
                               'delta_rudder': (np.deg2rad(-60),
                                                np.deg2rad(60)),  # rad
                               'delta_t': (0, 1)}  # non-dimensional, change in thrust

        """ Initial Coefficients"""
        # Aerodynamics
        self.CL, self.CD, self.Cm = 0.0, 0.0, 0.0
        self.CY, self.Cl, self.Cn = 0.0, 0.0, 0.0

        # Forces and Thrust
        self.Ct = 0.0

        self.gravity_forces = np.zeros(3)
        self.forces = np.zeros(3)
        self.moments = np.zeros(3)
        self.load_factor = 0.0

        # Velocity
        self.TAS = 0.0    # true airspeed 
        self.CAS = 0.0    # calibrated airspeed
        self.EAS = 0.0    # estimated airspeed
        self.Mach = 0.0   # mach number

        self.q_inf = 0.0 # dynamic pressure inf (Pa)

        # Angles
        self.alpha = 0.0        # AOA [rad]
        self.alpha_rate = 0.0   # AOA rate [rad/s]
        self.beta = 0.0         # sideslip [rad]
        self.q = 0.0            # pitch rate [rad/s]
        self.p = 0.0            # roll rate [rad/s]
        self.r = 0.0            # yaw rate [rad/s]

    @property
    def delta_elevator(self):
        return self.controls['delta_elevator']

    @property
    def delta_aileron(self):
        return self.controls['delta_aileron']

    @property
    def delta_rudder(self):
        return self.controls['delta_rudder']

    @property
    def delta_t(self):
        return self.controls['delta_t']

    # solve coefficient longitudinal matrix
    def _calc_long_coefficients(self):

        self._long_in[1] = self.alpha
        self._long_in[2] = (self.q*self.c_mean)/(2*self.TAS)
        self._long_in[3] = (self.alpha_rate*self.c_mean)/(2*self.TAS)
        self._long_in[4] = self.controls['delta_elevator']

        # @: infix operator for matrix multiplication
        self.CL, self.CD, self.Cm = self._long_matrix @ self._long_in

    # solve coefficient latitude matrix
    def _calc_lat_coefficients(self):

        self._lat_in[0] = self.beta
        self._lat_in[1] = (self.p*self.span)/(2*self.TAS)
        self._lat_in[2] = (self.r*self.span)/(2*self.TAS)
        self._lat_in[3] = self.delta_aileron
        self._lat_in[4] = self.delta_rudder

    # calculates forces and moments 
    def _calc_aero_forces_moments(self):
        q = self.q_inf
        S = self.S
        #c = self.chord
        b = self.span
        a = self.alpha
        self._calc_lat_coefficients()
        self._calc_long_coefficients()

        L = q*S*b*self.CL
        M = q*S*self.c_mean*self.Cm
        N = q*S*b*self.Cn
        Fx = q*S*(-self.CD*np.cos(a) + self.CL*np.sin(a))
        Fy = q*S*self.CY
        Fz = q*S*(-self.CD*np.sin(a) - self.CL*np.cos(a))

        return L,M,N,Fx,Fy,Fz

    def _calc_thrust(self):
        q = self.q_inf
        S = self.S
        a = self.alpha
        #self.Ct = 
        thrust = 0.001*(q*S*(self.CD*np.cos(a) - self.CL*np.sin(a)) + self.W*np.sin(a)) # convert to kN
        Ft = np.array([thrust,0,0])
        return Ft


    # abstract class from aircraft base class
    def calc_forces_and_moments(self,state,environment,controls):
        super().calc_forces_and_moments(state,environment,controls)
        L,M,N,Fx,Fy,Fz = self._calc_aero_forces_moments()
        Ft = self._calc_thrust()
        Fg = environment.gravity_vector*self.mass
        Fa = np.array([-Fx,Fy,-Fz])

        self.forces = Ft + Fg + Fa      # N
        self.moments = np.array([L,M,N])

        return self.forces, self.moments
