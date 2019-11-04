"""
Iteratively calculates the trim state of the input aircraft
"""
import copy
from math import sqrt,sin,cos,tan,atan
import numpy as np
from scipy.optimize import least_squares
from Earth_Models.flat_earth import EulerFlatEarth
from Vehicle_Physics._init_ import VehicleState, BodyVelocity, EulerAttitude
from Utility.constants import GRAVITY
from Utility.Transformations import wind2body

"""
calculates steady-state flight condition
aka: all rates are = 0 or constant, no acceleration
pos: vehicle position
psi: yaw angle [rad]
TAS: true airspeed
gamma: heading (default zero)
exclude: any control value that you dont want trimmed
verbose: if you want feedback from the least_squares method
-----0: no feedback, 1:display termination report, 2: display progress during iteration
returns: trimmed aircraft state, and trimmed controls
"""
def steady_state_trim(aircraft,environment,pos,psi,TAS,controls,gamma=0.0,turn_rate=0,exclude=None,verbose=0):
    """[Calculates steady state trim of vehicle]
    
    Arguments:
        aircraft {[aircraft]} -- [the vehicle in simulation]
        environment {[environment]} -- [environment where vehicle is flying]
        pos {[array]} -- [vehicle location in earth frame]
        psi {[float]} -- [yaw angle]
        TAS {[float]} -- [true airspeed]
        controls {[dict]} -- [controls to trim]
    
    Keyword Arguments:
        gamma {float} -- [flight path angle] (default: {0})
        turn_rate {int} -- [d(psi)/dt] (default: {0})
        exclude {[list]} -- [controls to not be trimmed] (default: {None})
        verbose {int} -- [least squares verbosity: 0-2 lvl of progress print outs] (default: {0})
    """
    
    # define initial state
    att0 = EulerAttitude(theta=0,phi=0,psi=psi)
    vel0 = BodyVelocity(u=TAS,v=0,w=0,attitude=att0)

    # define total state
    state0 = VehicleState(pos,att0,vel0)

    # make copy of environment and vehicle to not
    # change their state during trimming computation
    environment = copy.deepcopy(environment)
    aircraft = copy.deepcopy(aircraft)

    # update environment to the current state
    environment.update(state0)

    # solve for trim state
    system = EulerFlatEarth(time0=0,tot_state=state0)

    # initialize alpha and beta
    alpha0 = 0.05
    beta0 = 0.001*np.sign(turn_rate)

    # calculate the aerodynamics at this current state
    aircraft._calc_aerodynamics_2(TAS,alpha0,beta0,environment)


    # Initialize controls
    for control in aircraft.controls:
        if control not in controls:
            raise  ValueError("Control {} not given in initial_conditions: {}".format(control,controls))
        else:
            aircraft.controls[control] = controls[control]

    if exclude is None:
        exclude = []

    # choose the controls to be trimmed
    controls_to_trim = list(aircraft.controls.keys()-exclude)

    # set variables for optimization
    initial_guess = [alpha0,beta0]
    for control in controls_to_trim:
        initial_guess.append(controls[control])

    # sets bounds for each optimization variable
    lower_bounds = [-0.5,-0.25]
    upper_bounds = [+0.5,+0.25]
    for ii in controls_to_trim:
        lower_bounds.append(aircraft.control_limits[ii][0])
        upper_bounds.append(aircraft.control_limits[ii][1])
    bounds = (lower_bounds,upper_bounds)

    args = (system,aircraft,environment,controls_to_trim,gamma,turn_rate)

    # Trims
    results = least_squares(trimming_cost_func,x0=initial_guess,args=args,verbose=verbose,bounds=bounds)

    # residuals: final trim_function evaluation
    u_dot,v_dot,w_dot,p_dot,q_dot,r_dot = results.fun

    att = system.tot_state.attitude
    system.tot_state.acceleration.update([u_dot,v_dot,w_dot],att)
    system.tot_state.ang_accel.update([p_dot,q_dot,r_dot],att)

    trimmed_controls = controls
    for key,val in zip(controls_to_trim,results.x[2:]):
        trimmed_controls[key] = val

    return system.tot_state,trimmed_controls

def turn_coords_cons(turn_rate,alpha,beta,TAS,gamma=0):
    """[Calculates phi for corodinated turn]

    Arguments:
        turn_rate {[float]} -- [description]
        alpha {[float]} -- [description]
        beta {[float]} -- [description]
        TAS {[float]} -- [true airspeed]

    Keyword Arguments:
        gamma {int} -- [flight path angle] (default: {0})

    Returns:
        [float] -- [phi for the coordinated turn]
    """
    g0 = GRAVITY
    G = turn_rate*TAS/g0

    if abs(gamma) < 1e-8:
        phi = G*cos(beta)/(cos(alpha)-G*sin(alpha)*sin(beta))
        phi = atan(phi)
    else:
        a = 1-G*tan(alpha)*sin(beta)
        b = sin(gamma)/cos(beta)
        c = 1 + G** 2 * cos(beta)**2

        sq = sqrt(c * (1 -b *2) + G ** 2 * sin(beta)**2 )
        num = (a-b**2)+b*tan(alpha)*sq
        den = a**2-b**2*(1+c*tan(alpha)**2)

        phi = atan(G*cos(beta)/cos(alpha)*num/den)

    return phi

def turn_coords_horizontal_and_small_beta(turn_rate,alpha,TAS):
    """[calculates phi for coordinated turn given that gamma is equal to zero and small beta <<1]

    Arguments:
        turn_rate {[type]} -- [description]
        alpha {[type]} -- [description]
        TAS {[type]} -- [description]

    Returns:
        [float] -- [phi]
    """
    g0 = GRAVITY
    G = turn_rate*TAS/g0
    phi = G/cos(alpha)
    phi = atan(phi)
    return phi

def rate_of_climb_cons(gamma,alpha,beta,phi):
    """[calculates theta for the given Rate of climb,wind angles and roll angle]
    
    Arguments:
        gamma {[type]} -- [description]
        alpha {[type]} -- [description]
        beta {[type]} -- [description]
        phi {[type]} -- [description]
    """
    a = cos(alpha) * cos(beta)
    b = sin(phi) * sin(beta) + cos(phi) * sin(alpha) * cos(beta)
    sq = sqrt(a ** 2 - sin(gamma) ** 2 + b ** 2)
    theta = (a * b + sin(gamma) * sq) / (a ** 2 - sin(gamma) ** 2)
    theta = atan(theta)
    return theta

def trimming_cost_func(trimmed_params,system,aircraft,environment,controls2trim,gamma,turn_rate):

    alpha = trimmed_params[0]
    beta = trimmed_params[1]

    new_controls = {}
    for ii, control in enumerate(controls2trim):
        new_controls[control] = trimmed_params[ii + 2]

    # choose coordinated turn constrain equation
    if abs(turn_rate) < 1e-8:
        phi = 0
    else:
        phi = turn_coords_cons(turn_rate,alpha,beta,aircraft.TAS,gamma)

    # rate of climb constrain
    theta = rate_of_climb_cons(gamma,alpha,beta,phi)

    p = - turn_rate * sin(theta)
    q = turn_rate * sin(phi) * cos(theta)
    r = turn_rate * cos(theta) * cos(phi)

    u, v, w = wind2body((aircraft.TAS, 0, 0), alpha=alpha, beta=beta)

    psi = system.tot_state.attitude.psi
    system.tot_state.attitude.update([theta, phi, psi])
    attitude = system.tot_state.attitude

    system.tot_state.velocity.update([u, v, w], attitude)
    system.tot_state.ang_V.update([p, q, r], attitude)
    system.tot_state.acceleration.update([0, 0, 0], attitude)
    system.tot_state.ang_accel.update([0, 0, 0], attitude)

    out = system.steady_state_trim_func(system.tot_state,environment,aircraft,new_controls)

    return out