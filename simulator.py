import copy
import operator
import pandas as pd
import tqdm

"""
class to run the whole simulation shabang
stores time history of the simulation
applies progress bar
"""

class Simulation:

    _variable_dictionary = {

        'time': 'system.time',
        # environment
        'temperature': 'environment.temp',
        'pressure': 'environment.pressure',
        'rho': 'environment.rho',
        'a': 'environment.sos',
        # aircraft
        'Fx': 'aircraft.Fx',
        'Fy': 'aircraft.Fy',
        'Fz': 'aircraft.Fz',

        'Mx': 'aircraft.Mx',
        'My': 'aircraft.My',
        'Mz': 'aircraft.Mz',

        'TAS': 'aircraft.TAS',
        'Mach': 'aircraft.Mach',
        'q_inf': 'aircraft.q_inf',

        'alpha': 'aircraft.alpha',
        'beta': 'aircraft.beta',

        'rudder': 'aircraft.delta_rudder',
        'aileron': 'aircraft.delta_aileron',
        'elevator': 'aircraft.delta_elevator',
        'thrust': 'aircraft.delta_t',
        # system
        'x_earth': 'system.tot_state.position.x_earth',
        'y_earth': 'system.tot_state.position.y_earth',
        'z_earth': 'system.tot_state.position.z_earth',

        'height': 'system.tot_state.position.height',

        'psi': 'system.tot_state.attitude.psi',
        'theta': 'system.tot_state.attitude.theta',
        'phi': 'system.tot_state.attitude.phi',

        'u': 'system.tot_state.velocity.u',
        'v': 'system.tot_state.velocity.v',
        'w': 'system.tot_state.velocity.w',

        'v_north': 'system.tot_state.velocity.v_north',
        'v_east': 'system.tot_state.velocity.v_east',
        'v_down': 'system.tot_state.velocity.v_down',

        'p': 'system.tot_state.ang_V.p',
        'q': 'system.tot_state.ang_V.q',
        'r': 'system.tot_state.ang_V.r'

    }

    def __init__(self,aircraft,system,environment,controls,dt=0.01,save_vars=None):

        self.system = copy.deepcopy(system)
        self.aircraft = copy.deepcopy(aircraft)
        self.environment = copy.deepcopy(environment)

        self.system.update_simulation = self.update
        self.controls = controls
        self.dt = dt

        if not save_vars:
            self._save_vars = self._variable_dictionary
        
        self.results = {name: [] for name in self._save_vars}

    @property
    def time(self):
        return self.system.time

    def update(self,time,state):
        self.environment.update(state)
        controls = self._get_current_controls(time)
        self.aircraft.calc_forces_and_moments(state,self.environment,controls)
        return self

    def propagate(self,time):

        dt = self.dt
        half_dt = self.dt/2

        # set up progress bar
        bar = tqdm.tqdm(total=time, desc='time', initial=self.system.time)

        time_plus_half_dt = time + half_dt

        while self.system.time + dt < time_plus_half_dt:
            t = self.system.time
            self.environment.update(self.system.tot_state)
            controls = self._get_current_controls(t)
            self.aircraft.calc_forces_and_moments(self.system.tot_state,
                                                       self.environment,
                                                       controls)
            self.system.time_step(dt)
            self._save_time_step()
            bar.update(dt)

        bar.close()

        results = pd.DataFrame(self.results)
        results.set_index('time', inplace=True)

        return results

    def _save_time_step(self):

        # Saving the results according to the variable dictionary
        for var_name,value_pointer in self._save_vars.items():
            self.results[var_name].append((operator.attrgetter(value_pointer)(self)))

    def _get_current_controls(self,time):
        c = {c_name: c_fun(time) for c_name,c_fun in self.controls.items()}
        return c