from Vehicles.AD_Skyhawk import Skyhawk
from Environment.environment import Environment
from Environment.atmosphere import ISA1976
from Environment.wind import NoWind
from Environment.gravity import VerticalConstant
from Vehicle_Physics.position import EarthPosition
from Utility.trim_solver import steady_state_trim
from Utility.Signal_Generator import Constant
from simulator import Simulation



"""[Vehicle Data - format to access data]
"""
aircraft = Skyhawk()
"""
print(f"Aircraft mass: {aircraft.mass} kg")
print(f"Aircraft inertia tensor: \n {aircraft.inertia} kg/m²")
print(f"forces: {aircraft.forces} N")
print(f"moments: {aircraft.moments} N·m")
print(aircraft.controls)
print(aircraft.control_limits)
"""

"""[Environment set up and definition]
"""
atmosphere = ISA1976()
gravity = VerticalConstant()
wind = NoWind()
environment = Environment(atmosphere,gravity,wind)

pos = EarthPosition(x=0,y=0,height=10000)
psi = 0.0               #rad
TAS = 600                #true airspeed m/s
controls_init = {'delta_elevator': 0, 'delta_aileron': 0, 'delta_rudder': 0, 'delta_t': 0.0}

trimmed_state,trimmed_controls = steady_state_trim(aircraft,environment,pos,psi,TAS,controls_init)
print(f"Trimmed State:{trimmed_state}")
#print(f"Trimmed Controls: {trimmed_controls}")
environment.update(trimmed_state)
forces,moments = aircraft.calc_forces_and_moments(trimmed_state,environment,controls_init)
print(forces)
print(moments)
