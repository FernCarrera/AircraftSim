from Vehicles.AD_Skyhawk import Skyhawk

from Environment.atmosphere import ISA1976
from Environment.wind import NoWind
from Environment.gravity import VerticalConstant
from Environment.environment import Environment

from Utility.trim_solver import steady_state_trim

from Vehicle_Physics.position import EarthPosition

from Earth_Models.flat_earth import EulerFlatEarth

from Utility.Signal_Generator import Constant, Doublet
from simulator import Simulation

import matplotlib.pyplot as plt


aircraft = Skyhawk()
atmosphere = ISA1976()
gravity = VerticalConstant()
wind = NoWind()

environment = Environment(atmosphere, gravity, wind)

pos = EarthPosition(x=0, y=0, height=5000)
psi = 0.5  # rad
TAS = 45  # m/s
controls0 = {'delta_elevator': 0, 'delta_aileron': 0, 'delta_rudder': 0, 'delta_t': 0.5}

trimmed_state, trimmed_controls = steady_state_trim(
    aircraft,
    environment,
    pos,
    psi,
    TAS,
    controls0
)    

system = EulerFlatEarth(time0=0, tot_state=trimmed_state)

de0 = trimmed_controls['delta_elevator']

controls = controls = {
    'delta_elevator': Doublet(t_init=2, T=1, A=0.1, offset=de0),
    'delta_aileron': Constant(trimmed_controls['delta_aileron']),
    'delta_rudder': Constant(trimmed_controls['delta_rudder']),
    'delta_t': Constant(trimmed_controls['delta_t'])
}

sim = Simulation(aircraft, system, environment, controls, dt=0.3)
results_03 = sim.propagate(25)

#sim = Simulation(aircraft, system, environment, controls, dt=0.05)
#results_005 = sim.propagate(25)

kwargs = {'subplots': True,
          'sharex': True,
          'figsize': (12, 100)}

#ax = results_005.plot(marker='.', color='r', **kwargs)
#ax = resu  lts_03.plot(ax=ax, marker='x', color='k', ls='', **kwargs)
results_03.to_excel("output.xlsx")
#plt.show()