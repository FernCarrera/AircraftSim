"""[calcualtes vector of force and speed of airflow]

    - TAS: True airspeed. speed of the aircraft relative to the fluid its flying in
    - EAS: airspeed @ sea-level in which the dynamoc pressure is the same as the dunamic pressure at TAS/altitude
    - CAS: speed shown by airspeed  indicator after instrument/position error
"""

from math import asin,atan, sqrt
from Utility.constants import SL_DENSITY,SL_PRESSURE,SL_SOS,GAMMA_AIR,SL_TEMP

rho0 = SL_DENSITY
p0 = SL_PRESSURE
SOS = SL_SOS
gamma = GAMMA_AIR


def calc_alpha_beta_TAS(u,v,w):
    """[Calculate AOA,angle of sideslip and TAS from aero velocity in body frame]
    
    Arguments:
        u {[int/float]} -- [x-axis component of velocity [m/s]]
        v {[int/float]} -- [y-axis component of velocity [m/s]]
        w {[int/float]} -- [z-axis component of velocity [m/s]]
    """
    TAS = sqrt(u**2 + v**2 + w**2)
    alpha = atan(w/v)
    beta = asin(v/TAS)
    
    return alpha,beta,TAS

def calc_dynamic_pressure(rho,TAS):
    """[Calculates dynamic pressure]
    
    Arguments:
        rho {[float]} -- [air density]
        TAS {[float]} -- [true airspeed]
    """

    return 0.5*rho*TAS**2

def calc_viscosity(T):
    """[calculates viscosity of the air]
    
    Arguments:
        T {[float]} -- [temperature [K]]
    """
    visc0 = 1.176*1e-5
    b = 0.4042

    return visc0*(T/SL_TEMP)**(3/2)*((1+b)/((T/SL_TEMP)+b))
    
def TAS2EAS(TAS,rho):
    """[Convert TAS to EAS]
    
    Arguments:
        tas {[float]} -- [true aispeed m/s]
        rho {[float]} -- [airdensity kg/m**3]
    """
    EAS = TAS*sqrt(rho/rho0)
    return EAS

def EAS2TAS(EAS,rho):
    """[Convert EAS to TAS]
    
    Arguments:
        EAS {[float]} -- [equivalent aispeed m/s]
        rho {[float]} -- [airdensity kg/m**3]
    """
    TAS = EAS/sqrt(rho/rho0)
    return TAS

def TAS2CAS(TAS,p,rho):
    """[convert from TAS to CAS]
    
    Arguments:
        TAS {[float]} -- [true airspeed m/s]
        p {[float]} -- [pressure]
        rho {[float]} -- [air density kg/m**3]
    """

    a = sqrt(gamma*p/rho)
    var = (gamma-1)/gamma

    temp = (TAS**2 * (gamma - 1) / (2 * a**2) + 1) ** (1/var)
    temp = (temp - 1) * (p / p0)
    temp = (temp + 1) ** var - 1

    CAS = sqrt(2 * SOS ** 2 / (gamma - 1) * temp)
    return CAS
    



def stagnation_pressure(p,a,TAS):
    var = (gamma-1)/gamma
    M = TAS/SOS
    if M < 1:
        p_stag = 1 + (gamma-1)*(M**2)/2
        p_stag **=(1/var)
        p_stag *= p 
    else:
        p_stag = (gamma+1)**2 * M**2
        p_stag /= (4*gamma*(M**2) - 2*(gamma-1))
        p_stag **= (1/var)
        p_stag *= (1 - gamma + 2*gamma*(M**2)) / (gamma+1)
        p_stag *= p

    return p_stag