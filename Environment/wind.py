import numpy as np
from Vehicle_Physics.velocity import Velocity

"""
Need to make:
    - monte carlo wind sim
    - microburst wind sim
    - winds aloft data call to api??
        https://aviationweather.gov/windtemp
"""


class NoWind():
    def __init__(self):
        self.body = np.zeros([3],dtype=float)
        self.horizon = np.zeros([3],dtype=float)

    def update(self,state):
        pass


class WindShear(NoWind,Velocity):
    """[Wind shear model based on Frost & Bowles 
        "Wind shear terms in the equations of aircraft motion",
        Journal of Aircraft,vol 21 no.11, NOV 1984]
    
    Arguments:
        NoWind {[class]} -- [inherited from default wind class]
        Velocity {[class]} -- [inherited from velocity class]
    """

    def __init__(self,state):
        super().__init__()
        
    def calc_wind(self):
        """
        Wind acceleration  can be used to calculate changes
        in velocity and gamma in the flat-earth model, however changes in NED
        position require wind velocity - so an integration is required
        """

        Vn = self.v_north
        Ve = self.v_east
        Vd = self.v_down
        # wrt : with respect to
        D_wrt_d = 2.6   # [1/s], or 159nmi/hr/100ft
        D_wrt_e = 0.32      ## [1/s], or 19.6nmi/hr/100ft
        D_wrt_n = 0.32

        #wn_dot = (del_wn/del_n)*Vn + (del_wn/del_e)*Ve +(del_wn/del_d)*Vd + del_wn/dt
        #we_dot = (del_we/del_n)*Vn + (del_we/del_e)*Ve +(del_we/del_d)*Vd + del_we/dt
        #wd_dot = (D_wrt_n)*Vn + (D_wrt_e)*Ve +(D_wrt_d)*Vd + del_wd/dt

class MicroBurst(NoWind):

    def calc_MBurst(self,radius,height):
        """[microburst calculation]
        
        Arguments:
            radius {[float]} -- [radius of the microburst]
            height {[float]} -- [altitude of the microburst]
        """
        pass
        """
        fu = 2.0
        fr = 2.0
        D = 2000
        a = radius/400
        Wu = -fu*0.4*height/(a**4 + 10)
        b1 = (radius-0.5*D)/200
        Wr = fu*100/(b1**2 + 10)
        b2 = (radius+0.5*D)/200
        Wr -= fu*100/(b1**2 + 10)
        dWu = fu*0.4*(height+1)/(a*a*a*a+10) # perturb altitude
        pWuh = dWu-Wu
        a = (radius+1)/400 # perturb radius
        dWu = fu*0.4*height/(a*a*a*a+10)
        b = (radius+10.5*D)/200
        dWr = fr*100/(b*b+10)
        b = (radius+1+0.5*D)/200
        dWr = fr*100/(b*b+10)
        pWur = dWu-Wu
       
        pWrr = dWr-Wr
      

        return Wu,Wr
        """
        


