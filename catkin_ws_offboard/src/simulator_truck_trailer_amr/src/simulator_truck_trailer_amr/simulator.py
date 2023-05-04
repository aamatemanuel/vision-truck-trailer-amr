from rockit import *
from numpy import pi, cos, sin, tan
import yaml


def simulator_delta_init():
    '''Initialize simulator with steering angle as control input

    :return: sim_system_dyn
    :rtype: casadi.casadi.Function
    '''
    # define ocp
    ocp = Ocp()

    # Parameters
    with open('truck_trailer_para.yaml', 'r') as file:
        para = yaml.safe_load(file)

    L0 = para['truck']['L']
    M0 = para['truck']['M']
    L1 = para['trailer1']['L']
    M1 = para['trailer1']['M']

    # Trailer model
    theta1 = ocp.state()
    x1 = ocp.state()
    y1 = ocp.state()

    theta0 = ocp.state()

    delta0 = ocp.control()
    v0 = ocp.control()

    beta01 = theta0 - theta1

    dtheta0 = v0/L0*tan(delta0)
    dtheta1 = v0/L1*sin(beta01) - M0/L1*cos(beta01)*dtheta0
    v1 = v0*cos(beta01) + M0*sin(beta01)*dtheta0

    ocp.set_der(theta1, dtheta1)
    ocp.set_der(x1,     v1*cos(theta1))
    ocp.set_der(y1,     v1*sin(theta1))
    ocp.set_der(theta0, dtheta0)

    ocp.method(MultipleShooting(N=1, M=4, intg='rk'))

    ocp.solver('ipopt')

    # Get discretised dynamics as CasADi function
    sim_system_dyn = ocp.discrete_system()
    return sim_system_dyn


def simulator_omega_init(params):
    '''Initialize simulator with angular velocity as control input

    :param params: parameters of the truck and trailer
    :type params: dict

    :return: sim_system_dyn
    :rtype: casadi.casadi.Function
    '''
    # define ocp
    ocp = Ocp()

    # # Parameters
    # with open('truck_trailer_para.yaml', 'r') as file:
    #     para = yaml.safe_load(file)

    L0 = params['truck']['L']
    M0 = params['truck']['M']
    L1 = params['trailer1']['L']
    M1 = params['trailer1']['M']

    # Trailer model
    x1 = ocp.state()
    y1 = ocp.state()
    theta1 = ocp.state()

    theta0 = ocp.state()

    v0 = ocp.control()
    dtheta0 = ocp.control()

    beta01 = theta0 - theta1

    dtheta1 = v0/L1*sin(beta01) - M0/L1*cos(beta01)*dtheta0
    v1 = v0*cos(beta01) + M0*sin(beta01)*dtheta0

    ocp.set_der(theta1, dtheta1)
    ocp.set_der(x1,     v1*cos(theta1))
    ocp.set_der(y1,     v1*sin(theta1))
    ocp.set_der(theta0, dtheta0)

    ocp.method(MultipleShooting(N=1, M=4, intg='rk'))

    ocp.solver('ipopt')

    # Get discretized dynamics as CasADi function
    sim_system_dyn = ocp.discrete_system()
    return sim_system_dyn


def simulate(sim_system_dyn, x_current, u, dt):
    '''
    Simulate dynamics and update the current state

    sphinx docstring
    :param sim_system_dyn: simulator function
    :type sim_system_dyn: function

    :param x_current: current state of shape [theta1, x1, y1, theta0]
    :type x_current: np.array

    :param u: control input of shape [delta0 or dtheta0, v0]
    :type u: np.array

    :param dt: time step
    :type dt: float

    :return: next state
    :rtype: np.array
    '''
    x_next = sim_system_dyn(x0=x_current, u=u, T=dt)["xf"]

    return x_next
