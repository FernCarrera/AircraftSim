from abc import abstractmethod
from Utility.constants import R, SL_DENSITY, SL_PRESSURE, SL_TEMP, GRAVITY, GAMMA_AIR
from Utility.Transformations import geometric2geopotential
from numpy import exp,sqrt


class Atmosphere():

    def _init_(self):
        self._geopotential_alt = None   # m
        self._pressure_alt = None       # m
        self._Temp = None               # K
        self._pressure = None           # atm
        self._rho = None                # Density kg/m**3
        self._sos = None                 # Speed of sound m/s

    def update(self,state):

        # need to define function
        self._geopotential_alt = geometric2geopotential(state.position.height)

        T,p,rho,sos = self.getAtm(self._geopotential_alt)
        self._Temp = T
        self._pressure = p
        self._rho = rho
        self._sos = sos

    @property
    def temp(self):
        return self._Temp

    @property
    def pressure(self):
        return self._pressure

    @property
    def rho(self):
        return self._rho

    @property
    def sos(self):
        return self._sos

    # Allows for different gravity models/classes
    @abstractmethod
    def getAtm(self,alt):
        pass

class ISA1976(Atmosphere):

    def __init__(self):
        super().__init__()
        self._gamma = GAMMA_AIR
        self._R = R
        self._g0 = GRAVITY

        #1976 STD ATMOSPHERE TABULATED DATA
        self._h_ref = (0,11000,20000,32000,47000,51000,71000)
        self._T_ref = (288.15,216.65,228.65,270.65,270.65,214.65)
        self._P_ref = (101325,22632.1,5474.89,868.019,110.906,669389,3.95624)
        self._lapse = (-0.0065,0,0.001,0.0028,0,-0.0028,-0.002)

        self.alt = 0        # Current altitude, m
        self._Temp = self._T_ref[0]
        self._pressure = self._P_ref[0]
        self._rho = self.pressure/(self._R * self.temp)
        self._sos = sqrt(self._gamma*self._R*self.temp)

    
    def getAtm(self,alt):
        g0 = self._g0
        R = self._R
        gamma = self._gamma

        # Invalid entry check
        if alt < 0.0:
            raise ValueError(" Altitude Cannot be less than 0 m")

        # Troposphere
        elif self._h_ref[0] <= alt < self._h_ref[1]:
            T0 = self._T_ref[0]
            p0 = self._P_ref[0]
            alpha = self._lapse[0]

            T = T0 + alpha * alt
            p = p0 * (T0 / (T0 + alpha * alt)) ** (g0 / (R * alpha))

        # tropopause
        elif self._h_ref[1] <= alt < self._h_ref[2]:
            T0 = self._T_ref[1]
            p0 = self._P_ref[1]
            alt_err = alt - self._h_ref[1]
            T = T0
            p = p0 * exp(-g0 * alt_err / (R * T0))

        # stratosphere #1
        elif self._h_ref[2] <= alt < self._h_ref[3]:
            T0 = self._T_ref[2]
            p0 = self._P_ref[2]
            alpha = self._lapse[2]
            alt_err = alt - self._h_ref[2]

            T = T0 + alpha*alt_err
            p = p0*(T0/(T0 + alpha*alt_err))**(g0/(R*alpha))

        # stratosphere #2
        elif self._h_ref[3] <= alt < self._h_ref[4]:
            T0 = self._T_ref[3]
            p0 = self._P_ref[3]
            alpha = self._lapse[3]
            alt_err = alt - self._h_ref[3]

            T = T0 + alpha*alt_err
            p = p0*(T0/(T0 + alpha*alt_err))**(g0/(R*alpha))

        #stratopause
        elif self._h_ref[4] <= alt < self._h_ref[5]: 
            T0 = self._T_ref[4]
            p0 = self._P_ref[4]
            alt_err = alt - self._h_ref[4]

            T = T0
            p = p0*exp(-g0*alt_err/(R*T0))

        # mesosphere 1
        elif self._h_ref[5] <= alt < self._h_ref[6]: 
            T0 = self._T_ref[5]
            p0 = self._P_ref[5]
            alpha = self._lapse[5]
            alt_err = alt - self._h_ref[5]

            T = T0 + alpha*alt_err
            p = p0*(T0/(T0 + alpha*alt_err))**(g0/(R*alpha))

        # mesosphere 2
        elif self._h_ref[6] <= alt < self._h_ref[7]: 
            T0 = self._T_ref[6]
            p0 = self._P_ref[6]
            alpha = self._lapse[6]
            alt_err = alt - self._h_ref[6]

            T = T0 + alpha*alt_err
            p = p0*(T0/(T0+alpha*alt_err))**(g0/(R*alpha))

        else:
            raise ValueError("Altitude cannot be greater than{}m.".format(self._h_ref[7]))

        rho = p/(R*T)
        a = sqrt(gamma*R*T)
        return T,p,rho,a