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

Simple motion planning for vehicle with multiple trailers
"""

from rockit import *
import matplotlib.pyplot as plt
import numpy as np
from numpy import pi, cos, sin, tan
from plot_trailer import *
import yaml

show_figures = True
save_for_gif = False

ocp = Ocp(T=FreeTime(20.0))
N = 40
M = 1
refine = 10

# Parameters
with open('truck_trailer_para2.yaml', 'r') as file:
    para = yaml.safe_load(file)

L0 = para['truck']['L']
M0 = para['truck']['M']
W0 = para['truck']['W']
L1 = para['trailer1']['L']
M1 = para['trailer1']['M']
W1 = para['trailer1']['W']
L2 = para['trailer2']['L']
M2 = para['trailer2']['M']
W2 = para['trailer2']['W']

x2_t0 = 0.
y2_t0 = 0.
theta2_t0 = 0.
theta1_t0 = 0.
theta0_t0 = 0.

x2_tf = -2.
y2_tf = -2.
theta2_tf = -3*pi/4
theta1_tf = -3*pi/4
theta0_tf = -3*pi/4

# Trailer model
theta2 = ocp.state()
x2     = ocp.state()
y2     = ocp.state()

theta1 = ocp.state()
x1     = x2 + L2*cos(theta2) + M1*cos(theta1)
y1     = y2 + L2*sin(theta2) + M1*sin(theta1)

theta0 = ocp.state()
x0     = x1 + L1*cos(theta1) + M0*cos(theta0)
y0     = y1 + L1*sin(theta1) + M0*sin(theta0)

delta0 = ocp.control(order=1)
v0     = ocp.control(order=1)

beta01 = theta0 - theta1
beta12 = theta1 - theta2

dtheta0 = v0/L0*tan(delta0)
dtheta1 = v0/L1*sin(beta01) - M0/L1*cos(beta01)*dtheta0
v1 = v0*cos(beta01) + M0*sin(beta01)*dtheta0

dtheta2 = v1/L2*sin(beta12) - M1/L2*cos(beta12)*dtheta1
v2 = v1*cos(beta12) + M1*sin(beta12)*dtheta1

ocp.set_der(theta2, dtheta2)
ocp.set_der(x2,     v2*cos(theta2))
ocp.set_der(y2,     v2*sin(theta2))

ocp.set_der(theta1, dtheta1)
ocp.set_der(theta0, dtheta0)

# Initial constraints
ocp.subject_to(ocp.at_t0(x2) == x2_t0)
ocp.subject_to(ocp.at_t0(y2) == y2_t0)
ocp.subject_to(ocp.at_t0(theta2) == theta2_t0)
ocp.subject_to(ocp.at_t0(theta1) == theta1_t0)
ocp.subject_to(ocp.at_t0(theta0) == theta0_t0)

# Final constraint
ocp.subject_to(ocp.at_tf(x2) == x2_tf)
ocp.subject_to(ocp.at_tf(y2) == y2_tf)
ocp.subject_to(ocp.at_tf(theta2) == -3*pi/4)
ocp.subject_to(ocp.at_tf(beta12) == theta1_tf - theta2_tf)
ocp.subject_to(ocp.at_tf(beta01) == theta0_tf - theta1_tf)

# Initial guess
ocp.set_initial(theta0, .1)
ocp.set_initial(theta1, 0)
ocp.set_initial(theta2, 0)
ocp.set_initial(v0,     -.2)
ocp.set_initial(x2,     np.linspace(x2_t0, x2_tf, N))
ocp.set_initial(y2,     np.linspace(y2_t0, y2_tf, N))
 
# Path constraints
ocp.subject_to(-.2 <= (v0 <= .2))
ocp.subject_to(-1 <= (ocp.der(v0) <= 1))

ocp.subject_to(-pi/6 <= (delta0 <= pi/6))
ocp.subject_to(-pi/10 <= (ocp.der(delta0) <= pi/10))

ocp.subject_to(-pi/2 <= (beta01 <= pi/2))
ocp.subject_to(-pi/2 <= (beta12 <= pi/2))

# Minimal time
ocp.add_objective(ocp.T)
ocp.add_objective(ocp.integral(beta01**2))
ocp.add_objective(ocp.integral(beta12**2))

# Pick a solution method
options = { "expand": True,
			"verbose": False,
			"print_time": True,
			"error_on_fail": True,
			"ipopt": {	"linear_solver": "ma57",
						"tol":1e-8}}
ocp.solver('ipopt',options)

# Make it concrete for this ocp
ocp.method(MultipleShooting(N=N,M=M,intg='rk'))

# Solve
sol = ocp.solve()
import pdb; pdb.set_trace()

# Get solution
ts, theta2 = sol.sample(theta2, grid='integrator', refine=refine)
ts, x2     = sol.sample(x2,     grid='integrator', refine=refine)
ts, y2     = sol.sample(y2,     grid='integrator', refine=refine)

ts, theta1 = sol.sample(theta1, grid='integrator', refine=refine)
ts, x1     = sol.sample(x1,     grid='integrator', refine=refine)
ts, y1     = sol.sample(y1,     grid='integrator', refine=refine)

ts, theta0 = sol.sample(theta0, grid='integrator', refine=refine)
ts, x0     = sol.sample(x0,     grid='integrator', refine=refine)
ts, y0     = sol.sample(y0,     grid='integrator', refine=refine)

ts, delta0 = sol.sample(delta0, grid='integrator', refine=refine)
ts, v0     = sol.sample(v0,     grid='integrator', refine=refine)

if show_figures:
	# Show results
	from pylab import *

	plt.figure(1)
	ax1 = plt.subplot(1, 1, 1)
	ax1.axis('equal')

	ax1.plot(x0, y0, color='grey')
	ax1.plot(x1, y1, color='r')
	ax1.plot(x2, y2, color='g')

	plt.figure(2)
	ax21 = plt.subplot(1, 2, 1)
	ax22 = plt.subplot(1, 2, 2)
	ax21.plot(ts, delta0)
	ax22.plot(ts, v0)

	for k in range(N*refine + 1):
		x0s     = x0[k]
		y0s     = y0[k]
		theta0s = theta0[k]
		x1s     = x1[k]
		y1s     = y1[k]
		theta1s = theta1[k]
		x2s     = x2[k]
		y2s     = y2[k]
		theta2s = theta2[k]
		delta0s = delta0[k]

		truck           = vehic_to_plot(ax1, x0s, y0s, theta0s, W0/2,  W0/2,      L0, M0, color='grey')
		truck_steer     = wheel_to_plot(ax1, x0s, y0s, theta0s,   L0,     0, delta0s,     color='k')
		truck_fixed_1   = wheel_to_plot(ax1, x0s, y0s, theta0s,    0,  W0/2,       0,     color='k')
		truck_fixed_2   = wheel_to_plot(ax1, x0s, y0s, theta0s,    0, -W0/2,       0,     color='k')
		truck_xy        = ax1.plot(x0s, y0s, 'x', color='grey')

		trailer1         = vehic_to_plot(ax1, x1s, y1s, theta1s, W1/2,  W1/2,  .8*L1, M1, color='r')
		trailer1_fixed_1 = wheel_to_plot(ax1, x1s, y1s, theta1s,    0,  W1/2,      0,     color='k')
		trailer1_fixed_2 = wheel_to_plot(ax1, x1s, y1s, theta1s,    0, -W1/2,      0,     color='k')
		trailer1_xy      = ax1.plot(x1s, y1s, 'x', color='r')

		trailer2         = vehic_to_plot(ax1, x2s, y2s, theta2s, W2/2,  W2/2,  .8*L2, M2, color='g')
		trailer2_fixed_1 = wheel_to_plot(ax1, x2s, y2s, theta2s,    0,  W2/2,      0,     color='k')
		trailer2_fixed_2 = wheel_to_plot(ax1, x2s, y2s, theta2s,    0, -W2/2,      0,     color='k')
		trailer2_xy      = ax1.plot(x2s, y2s, 'x', color='r')

		coupling1     = vert_single(x0s, y0s, theta0s, -M0, 0)
		coupling1_xy  = ax1.plot([x1s, coupling1[0][0]], [y1s, coupling1[0][1]], '-', color='k')
		coupling1_dot = ax1.plot(coupling1[0][0], coupling1[0][1], 'o', color='k')	
		coupling2     = vert_single(x1s, y1s, theta1s, -M1, 0)
		coupling2_xy  = ax1.plot([x2s, coupling2[0][0]], [y2s, coupling2[0][1]], '-', color='k')
		coupling2_dot = ax1.plot(coupling2[0][0], coupling2[0][1], 'o', color='k')	

		if save_for_gif:
			png_name = 'trailer'+str(k)+'.png'
			plt.savefig(png_name)

		pause(.01)
		if k < N*refine:
			truck.pop(0).remove()
			truck_steer.pop(0).remove()
			truck_fixed_1.pop(0).remove()
			truck_fixed_2.pop(0).remove()
			truck_xy.pop(0).remove()
			trailer1.pop(0).remove()
			trailer1_fixed_1.pop(0).remove()
			trailer1_fixed_2.pop(0).remove()
			trailer1_xy.pop(0).remove()
			trailer2.pop(0).remove()
			trailer2_fixed_1.pop(0).remove()
			trailer2_fixed_2.pop(0).remove()
			trailer2_xy.pop(0).remove()
			coupling1_xy.pop(0).remove()
			coupling1_dot.pop(0).remove()
			coupling2_xy.pop(0).remove()
			coupling2_dot.pop(0).remove()

	show(block=True)
