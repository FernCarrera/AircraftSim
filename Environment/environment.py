
"""
Defines the environment the vehicle is flying in
Atmosphere, gravity, and wind
"""

class Environment():

    def __init__(self,atmosphere,gravity,wind):
        self.atmosphere = atmosphere
        self.gravity = gravity
        self.wind = wind


    # Get atmosphere qualities from atmosphere model
    @property
    def temp(self):
        return self.atmosphere.temp

    @property
    def pressure(self):
        return self.atmosphere.pressure

    @property
    def rho(self):
        return self.atmosphere.rho

    @property
    def sos(self):
        return self.atmosphere.sos

    # get gravity qualities from gravity model
    @property
    def gravity_magnitude(self):
        return self.gravity.magnitude

    @property
    def gravity_vector(self):
        return self.gravity.vector

    # get wind quality from wind model
    @property
    def body_wind(self):
        return self.wind.body

    @property
    def horizon_wind(self):
        return self.wind.horizon

    # update the models with the vehicles position
    def update(self,state):
        self.atmosphere.update(state)
        self.gravity.update(state)
        self.wind.update(state)