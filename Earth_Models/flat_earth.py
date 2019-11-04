"""
3dof earth models:
non-rotating spherical earth, rotating spherical earth
flat earth
"""
from Utility.Integration import VehicleIntegration
from Vehicle_Physics._init_ import EarthPosition,EulerAngAccel,EulerAngRates,EulerAttitude,BodyAccel,BodyAngVel,BodyVelocity,VehicleState,BodyAngAccel
import numpy as np
from numpy import sin,cos

"""
Flat-Earth model
"""
class EulerFlatEarth(VehicleIntegration):

    def func(self,t,x):

        self._update_tot_state(x,self.state_vector_derivatives)
        updated_sim = self.update_simulation(t,self.tot_state)
        
        # is t used as an index?
        mass = updated_sim.tot_state.aircraft.mass
        inertia = updated_sim.tot_state.aircraft.inertia
        forces = updated_sim.tot_state.aircraft.forces
        moments = updated_sim.tot_state.aircraft.moments

        out = _system_equations(t,x,mass,inertia,forces,moments)
        return out
    
    
    def steady_state_trim_func(self,tot_state,environment,aircraft,controls):
        """[Applies steeady state trim to the vehicle]
        
        Arguments:
            tot_state {[array]} -- [total state of the vehicle]
            environment {[type]} -- [environment where vehicle is flying]
            aircraft {[type]} -- [vehicle type]
            controls {[type]} -- [description]
        """
        environment.update(tot_state)
        aircraft.calc_forces_and_moments(tot_state,environment,controls)

        mass = aircraft.mass
        inertia = aircraft.inertia
        forces = aircraft.forces
        moments = aircraft.moments

        t0 = 0
        state0 = self._get_vehicle_state_vector(tot_state)
        out = _system_equations(t0,state0,mass,inertia,forces,moments)
        
        return out[:6]

    def _update_tot_state(self,state,state_derivatives):

        self.tot_state.position.update(state[9:12])
        self.tot_state.attitude.update(state[6:9])
        att = self.tot_state.attitude
        self.tot_state.velocity.update(state[0:3],att)
        self.tot_state.ang_V.update(state[3:6],att)
        self.tot_state.acceleration.update(state_derivatives[0:3],att)
        self.tot_state.ang_accel.update(state_derivatives[3:6],att)

    def _adapt_tot_state(self,tot_state):
        pos = EarthPosition(tot_state.position.x_earth,
                            tot_state.position.y_earth,
                            tot_state.position.height,
                            tot_state.position.lat,
                            tot_state.position.lon)

        att = EulerAttitude(tot_state.attitude.theta,
                            tot_state.attitude.phi,
                            tot_state.attitude.psi)

        vel = BodyVelocity(tot_state.velocity.u,
                           tot_state.velocity.v,
                           tot_state.velocity.w,
                           att)

        ang_vel = BodyAngVel(tot_state.ang_V.p,
                                      tot_state.ang_V.q,
                                      tot_state.ang_V.r,
                                      att)

        accel = BodyAccel(tot_state.acceleration.u_dot,
                                 tot_state.acceleration.v_dot,
                                 tot_state.acceleration.w_dot,
                                 att)

        ang_accel = BodyAngAccel(tot_state.ang_accel.p_dot,
                                            tot_state.ang_accel.q_dot,
                                            tot_state.ang_accel.r_dot,
                                            att)

        tot_state = VehicleState(pos, att, vel, ang_vel, accel, ang_accel)
        return tot_state

    def _get_vehicle_state_vector(self,tot_state):
        state = np.array([
    
                tot_state.velocity.u,
                tot_state.velocity.v,
                tot_state.velocity.w,
                tot_state.ang_V.p,
                tot_state.ang_V.q,
                tot_state.ang_V.r,
                tot_state.attitude.theta,
                tot_state.attitude.phi,
                tot_state.attitude.psi,
                tot_state.position.x_earth,
                tot_state.position.y_earth,
                tot_state.position.z_earth
        ])

        return state
"""
Solving the euler flat earth model equations
"""
def _system_equations(time,state_vector,mass,inertia,forces,moments):
    
    # Intertias
    Ix = inertia[0,0]
    Iy = inertia[1,1]
    Iz = inertia[2,2]
    Jxz = -inertia[0,2]

    # Forces
    Fx, Fy, Fz = forces
    L, M, N = moments

    u, v, w = state_vector[0:3]
    p, q, r = state_vector[3:6]
    theta, phi, psi = state_vector[6:9]

    # Linear Momentum Equations
    dudt = Fx/mass + r*v-q*w
    dvdt = Fy/mass - r*u+p*w
    dwdt = Fz/mass + q*u-p*v

    # angular momentum equations
    dpdt = (L * Iz + N * Jxz - q * r * (Iz ** 2 - Iz * Iy + Jxz ** 2) + p * q * Jxz * (Ix + Iz - Iy)) / (Ix * Iz - Jxz ** 2)
    dqdt = (M + (Iz - Ix) * p * r - Jxz * (p ** 2 - r ** 2)) / Iy
    drdt = (L * Jxz + N * Ix + p * q * (Ix ** 2 - Ix * Iy + Jxz ** 2) - q * r * Jxz * (Iz + Ix - Iy)) / (Ix * Iz - Jxz ** 2)

    # linear kinematic equations
    dxdt = (cos(theta) * cos(psi) * u + (sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi)) * v + (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)) * w)
    dydt = (cos(theta) * sin(psi) * u + (sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi)) * v + (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)) * w)
    dzdt = -u * sin(theta) + v * sin(phi) * cos(theta) + w * cos(phi) * cos(theta)

    # angulat kinematic equations
    dtheta_dt = q * cos(phi) - r * sin(phi)
    dphi_dt = p + (q * sin(phi) + r * cos(phi)) * np.tan(theta)
    dpsi_dt = (q * sin(phi) + r * cos(phi)) / cos(theta)

    return np.array([dudt, dvdt, dwdt, dpdt, dqdt, drdt, dtheta_dt,dphi_dt, dpsi_dt, dxdt, dydt, dzdt])