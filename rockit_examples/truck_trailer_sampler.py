#
#     This file is part of rockit.
#
#     rockit -- Rapid Optimal Control Kit
#     Copyright (C) 2019 MECO, KU Leuven. All rights reserved.
#
#     Rockit is free software; you can redistribute it and/or
#     modify it under the terms of the GNU Lesser General Public
#     License as published by the Free Software Foundation; either
#     version 3 of the License, or (at your option) any later version.
#
#     Rockit is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#     Lesser General Public License for more details.
#
#     You should have received a copy of the GNU Lesser General Public
#     License along with CasADi; if not, write to the Free Software
#     Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
#
#

"""
Motion planning
===============

Simple motion planning for vehicle with trailer
"""

from rockit import *
import matplotlib.pyplot as plt
import numpy as np
from numpy import pi, cos, sin, tan
from casadi import vertcat
from plot_trailer import *
from simulator import *
import yaml
import time

show_figures = True
save_for_gif = False

ocp = Ocp(T=FreeTime(20.0))
N = 20
M = 1
refine = 1
Ts = 0.04
Tupdate = 0.5

# Parameters
with open('truck_trailer_para.yaml', 'r') as file:
    para = yaml.safe_load(file)

L0 = 0.3375
M0 = 0.1
W0 = 0.2
L1 = 0.3
M1 = 0.06
W1 = 0.2

x1_t0 = 0.
y1_t0 = 0.
theta1_t0 = 0.
theta0_t0 = 0.

x1_tf = 1.
y1_tf = 0.
theta1_tf = 0.
theta0_tf = 0.

# Trailer model
theta1 = ocp.state()
x1     = ocp.state()
y1     = ocp.state()

theta0 = ocp.state()
x0     = x1 + L1*cos(theta1) + M0*cos(theta0)
y0     = y1 + L1*sin(theta1) + M0*sin(theta0)

delta0 = ocp.control(order=1)
v0     = ocp.control(order=1)

beta01 = theta0 - theta1

dtheta0 = v0/L0*tan(delta0)
dtheta1 = v0/L1*sin(beta01) - M0/L1*cos(beta01)*dtheta0
v1 = v0*cos(beta01) + M0*sin(beta01)*dtheta0

ocp.set_der(theta1, dtheta1)
ocp.set_der(x1,     v1*cos(theta1))
ocp.set_der(y1,     v1*sin(theta1))

ocp.set_der(theta0, dtheta0)

# Initial constraints
ocp.subject_to(ocp.at_t0(x1) == x1_t0)
ocp.subject_to(ocp.at_t0(y1) == y1_t0)
ocp.subject_to(ocp.at_t0(theta1) == theta1_t0)
ocp.subject_to(ocp.at_t0(theta0) == theta0_t0)

# Final constraint
ocp.subject_to(ocp.at_tf(x1) == x1_tf)
ocp.subject_to(ocp.at_tf(y1) == y1_tf)
ocp.subject_to(ocp.at_tf(theta1) == theta1_tf)
ocp.subject_to(ocp.at_tf(beta01) == theta0_tf - theta1_tf)

# Initial guess
ocp.set_initial(theta0, .1)
ocp.set_initial(theta1, 0)
ocp.set_initial(v0,    -.2)
ocp.set_initial(x1,     np.linspace(x1_t0, x1_tf, N))
ocp.set_initial(y1,     np.linspace(y1_t0, y1_tf, N))

# Path constraints
ocp.subject_to(-.2 <= (v0 <= .2))
ocp.subject_to(-1 <= (ocp.der(v0) <= 1))

ocp.subject_to(-pi/6 <= (delta0 <= pi/6))
ocp.subject_to(-pi/10 <= (ocp.der(delta0) <= pi/10))

ocp.subject_to(-pi/2 <= (beta01 <= pi/2))

# Minimal time
ocp.add_objective(ocp.T)
ocp.add_objective(ocp.integral(beta01**2))

# Pick a solution method
options = { "expand": True,
			"verbose": False,
			"print_time": True,
			"error_on_fail": True,
			"ipopt": {	"linear_solver": "ma57",
						"tol": 1e-8}}
ocp.solver('ipopt',options)

# Make it concrete for this ocp
ocp.method(MultipleShooting(N=N,M=M,intg='rk'))

T_v      = ocp.value(ocp.T)
theta1_s = ocp.sample(theta1, grid='control')[1]
x1_s     = ocp.sample(x1, 	  grid='control')[1]
y1_s     = ocp.sample(y1, 	  grid='control')[1]
theta0_s = ocp.sample(theta0, grid='control')[1]
delta0_s = ocp.sample(delta0, grid='control')[1]
v0_s     = ocp.sample(v0, 	  grid='control')[1]

sampler  = ocp.sampler([theta1, x1, y1, theta0, x0, y0, delta0, v0, dtheta0])

solve_ocp = ocp.to_function('solve_ocp',
                            [T_v, theta1_s, x1_s, y1_s, theta0_s, delta0_s, v0_s],
                            [T_v, theta1_s, x1_s, y1_s, theta0_s, delta0_s, v0_s, ocp.gist])

# Solve func
t0 = time.time()
T_sol, theta1_sol, x1_sol, y1_sol, theta0_sol, delta0_sol, v0_sol, gist_sol = solve_ocp(0, 0, 0, 0, 0, 0, 0)
# t_ctrl = np.arange(0., T_sol, Ts)
t1 = time.time()
t_ctrl = np.arange(0., Tupdate, Ts)  # Only sample first second.
t2 = time.time()
[theta1_ctrl, x1_ctrl, y1_ctrl, theta0_ctrl, x0_ctrl, y0_ctrl, delta0_ctrl, v0_ctrl, dtheta0_ctrl] = sampler(gist_sol, t_ctrl)
t3 = time.time()

print('OCP solving time: ' + str(t1-t0))
print('Sampling solution time: ' + str(t3-t2))




# =============================================================================
# Plotting
# --------
Nsim = len(t_ctrl)
if show_figures:
    # Show results
    from pylab import *

    plt.figure(1)
    ax1 = plt.subplot(1, 1, 1)
    ax1.axis('equal')

    ax1.plot(x0_ctrl, y0_ctrl, color='grey')
    ax1.plot(x1_ctrl, y1_ctrl, color='r')
    start, = ax1.plot(x1_ctrl[0], y1_ctrl[0], 'o', color='r',markersize=12)
    stop, = ax1.plot(x1_ctrl[-1], y1_ctrl[-1], 'x', color='r',markersize=12)
    vehic_to_plot(ax1, x1_ctrl[0],  y1_ctrl[0],  theta1_ctrl[0],  W1/2, W1/2, L1, M1, color='r')
    vehic_to_plot(ax1, x1_ctrl[-1], y1_ctrl[-1], theta1_ctrl[-1], W1/2, W1/2, L1, M1, color='r')
    ax1.set_xlabel('x [m]')
    ax1.set_ylabel('y [m]')

    ax1.legend([start, stop], ['initial', 'final'])

    plt.figure(2)
    ax21 = plt.subplot(1, 2, 1)
    ax22 = plt.subplot(1, 2, 2)
    ax21.plot(t_ctrl, delta0_ctrl)
    ax22.plot(t_ctrl, v0_ctrl)

    for k in range(Nsim-1):
    	x0s     = x0_ctrl[k]
    	y0s     = y0_ctrl[k]
    	theta0s = theta0_ctrl[k]
    	x1s     = x1_ctrl[k]
    	y1s     = y1_ctrl[k]
    	theta1s = theta1_ctrl[k]
    	delta0s = delta0_ctrl[k]

    	truck           = vehic_to_plot(ax1, x0s, y0s, theta0s, W0/2,  W0/2,      L0, M0, color='grey')
    	truck_steer     = wheel_to_plot(ax1, x0s, y0s, theta0s,   L0,     0, delta0s,     color='k')
    	truck_fixed_1   = wheel_to_plot(ax1, x0s, y0s, theta0s,    0,  W0/2,       0,     color='k')
    	truck_fixed_2   = wheel_to_plot(ax1, x0s, y0s, theta0s,    0, -W0/2,       0,     color='k')
    	truck_xy        = ax1.plot(x0s, y0s, 'x', color='grey')

    	trailer         = vehic_to_plot(ax1, x1s, y1s, theta1s, W1/2,  W1/2,   .8*L1, M1, color='r')
    	trailer_fixed_1 = wheel_to_plot(ax1, x1s, y1s, theta1s,    0,  W1/2,       0,     color='k')
    	trailer_fixed_2 = wheel_to_plot(ax1, x1s, y1s, theta1s,    0, -W1/2,       0,     color='k')
    	trailer_xy      = ax1.plot(x1s, y1s, 'x', color='r')

    	coupling     = vert_single(x0s, y0s, theta0s, -M0, 0)
    	coupling_xy  = ax1.plot([x1s, coupling[0][0]], [y1s, coupling[0][1]], '-', color='k')
    	coupling_dot = ax1.plot(coupling[0][0], coupling[0][1], 'o', color='k')

    	if save_for_gif:
    		png_name = 'trailer'+str(k)+'.png'
    		plt.savefig(png_name)

    	pause(.001)
    	if k < Nsim-1:
    		truck.pop(0).remove()
    		truck_steer.pop(0).remove()
    		truck_fixed_1.pop(0).remove()
    		truck_fixed_2.pop(0).remove()
    		truck_xy.pop(0).remove()
    		trailer.pop(0).remove()
    		trailer_fixed_1.pop(0).remove()
    		trailer_fixed_2.pop(0).remove()
    		trailer_xy.pop(0).remove()
    		coupling_xy.pop(0).remove()
    		coupling_dot.pop(0).remove()

    show(block=True)
