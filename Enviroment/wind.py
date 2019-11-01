import numpy as np

"""
Need to make:
    - monte carlo wind sim
    - microburst wind sim
"""


class NoWind():
    def __init__(self):
        self.body = np.zeros([3],dtype=float)
        #horizon air

    def update(self,state):
        pass