from Vehicles.AD_Skyhawk import Skyhawk
from Environment.environment import Environment
from Environment.atmosphere import ISA1976
from Environment.gravity import VerticalConstant
from Environment.wind import NoWind
from Vehicle_Physics.position import EarthPosition
from Earth_Models.flat_earth import EulerFlatEarth
from simulator import Simulation
from Utility.trim_solver import steady_state_trim

from Utility.Signal_Generator import Constant, Doublet


def test_simulation():

    atmosphere = ISA1976()
    gravity = VerticalConstant()
    wind = NoWind()

    environment = Environment(atmosphere, gravity, wind)
    aircraft = Skyhawk()

    initial_position = EarthPosition(0, 0, 1000)

    controls_0 = {'delta_elevator': 0.05,
                  'delta_aileron': 0,
                  'delta_rudder': 0,
                  'delta_t': 0.5,
                  }

    trimmed_state, trimmed_controls = steady_state_trim(
        aircraft, environment, initial_position, psi=1, TAS=50,
        controls=controls_0
    )

    system = EulerFlatEarth(time0=0, tot_state=trimmed_state)

    controls = {
        'delta_elevator': Doublet(2, 1, 0.1,
                                  trimmed_controls['delta_elevator']),
        'delta_aileron': Constant(trimmed_controls['delta_aileron']),
        'delta_rudder': Constant(trimmed_controls['delta_rudder']),
        'delta_t': Constant(trimmed_controls['delta_t'])
    }

    simulation = Simulation(aircraft, system, environment, controls)
    simulation.propagate(10)

if __name__=="__main__":
    test_simulation()

