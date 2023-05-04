from rockit import *
from numpy import pi, cos, sin, tan
import yaml


def simulator_delta_init():
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
    x1     = ocp.state()
    y1     = ocp.state()

    theta0 = ocp.state()

    delta0 = ocp.control()
    v0     = ocp.control()

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


def simulator_omega_init():
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
    theta1  = ocp.state()
    x1      = ocp.state()
    y1      = ocp.state()

    theta0  = ocp.state()

    dtheta0 = ocp.control()
    v0      = ocp.control()

    beta01 = theta0 - theta1

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


def simulator(sim_system_dyn, x_current, u, dt):
    """
    init with simulator_delta_init or simulator_omega_init
    states x
        theta1
        x1
        y1
        theta0

    controls u
        delta0 or dtheta0
        v0
    """

    # Simulate dynamics and update the current state
    x_next = sim_system_dyn(x0=x_current, u=u, T=dt)["xf"]

    return x_next
