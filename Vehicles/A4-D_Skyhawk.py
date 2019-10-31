import numpy as np
from Vehicles.aircraft import Aircraft


class Skyhawk(Aircraft):

    def __init__(self):
        self.mass = 7900                                # kg
        self.inertia = np.diag([10970,35100,39600])     # kg*m**2
        # Ixz = 1760

        # geometry
        self.S = 24.16                                  # m**2
        self.chord = 4.72                               # m
        self.span = 8.38                                # m

        