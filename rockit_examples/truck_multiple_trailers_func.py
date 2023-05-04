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
from casadi import vertcat, sumsqr
from plot_trailer import *

ocp = Ocp(T=FreeTime(20.0))
N = 50
refine = 10

# Parameters
L1 = .4 #m
M1 = .1 #m
L2 = .2 #m
M2 = .1 #m
L3 = .8 #m
M3 = .0 #m
width = .2 #m

# Obstacle
obs_x0 = 1
obs_y0 = -3 
obs_r0 = 1

# Trailer model
theta3 = ocp.state()
x3     = ocp.state()
y3     = ocp.state()

theta2 = ocp.state()
x2     = x3 + L3*cos(theta3) + M2*cos(theta2)
y2     = y3 + L3*sin(theta3) + M2*sin(theta2)

theta1 = ocp.state()
x1     = x2 + L2*cos(theta2) + M1*cos(theta1)
y1     = y2 + L2*sin(theta2) + M1*sin(theta1)

delta1 = ocp.control(order=0)
v1     = ocp.control(order=0)

beta12 = theta1 - theta2
beta23 = theta2 - theta3

dtheta1 = v1/L1*tan(delta1)
dtheta2 = v1/L2*sin(beta12) - M1/L2*cos(beta12)*dtheta1
v2 = v1*cos(beta12) + M1*sin(beta12)*dtheta1

dtheta3 = v2/L3*sin(beta23) - M2/L3*cos(beta23)*dtheta2
v3 = v2*cos(beta23) + M2*sin(beta23)*dtheta2

ocp.set_der(theta3, dtheta3)
ocp.set_der(x3,     v3*cos(theta3))
ocp.set_der(y3,     v3*sin(theta3))

ocp.set_der(theta2, dtheta2)
ocp.set_der(theta1, dtheta1)

# Initial constraints
ocp.subject_to(ocp.at_t0(x3) == 0)
ocp.subject_to(ocp.at_t0(y3) == 0)
ocp.subject_to(ocp.at_t0(theta3) == 0)
ocp.subject_to(ocp.at_t0(theta2) == 0)
ocp.subject_to(ocp.at_t0(theta1) == 0)

# Final constraint
x3_tf = ocp.parameter()
y3_tf = ocp.parameter()
theta3_tf = ocp.parameter()
ocp.set_value(x3_tf, -2)
ocp.set_value(y3_tf, -2)
ocp.set_value(theta3_tf, -3*pi/4)

ocp.subject_to(ocp.at_tf(x3) == x3_tf)
ocp.subject_to(ocp.at_tf(y3) == y3_tf)
ocp.subject_to(ocp.at_tf(theta3) == theta3_tf)
ocp.subject_to(ocp.at_tf(beta23) == 0)
ocp.subject_to(ocp.at_tf(beta12) == 0)

# Initial guess
ocp.set_initial(theta1, .1)
ocp.set_initial(theta2, 0)
ocp.set_initial(theta3, 0)
ocp.set_initial(v1,     -1)
ocp.set_initial(x3,     np.linspace(0,-2,N))
ocp.set_initial(y3,     np.linspace(0,-2,N))
 
# Path constraints
ocp.subject_to(-1 <= (v1 <= 1))
# ocp.subject_to(-1 <= (ocp.der(v1) <= 1))

ocp.subject_to(-pi/6 <= (delta1 <= pi/6))
# ocp.subject_to(-pi/10 <= (ocp.der(delta1) <= pi/10))

ocp.subject_to(-pi/2 <= (beta12 <= pi/2))
ocp.subject_to(-pi/2 <= (beta23 <= pi/2))

# Minimal time
ocp.add_objective(ocp.T)
#ocp.add_objective(ocp.integral(beta12**2))
#ocp.add_objective(ocp.integral(beta23**2))

# Pick a solution method
options = { "expand": True,
			"verbose": False,
			"print_time": True,
			"error_on_fail": False,
			"ipopt": {	"linear_solver": "ma57"}}
ocp.solver('ipopt',options)

# Make it concrete for this ocp
ocp.method(MultipleShooting(N=N,M=1,intg='rk'))

# Obstacle
obs_x = ocp.parameter()
obs_y = ocp.parameter()
obs_r = ocp.parameter()
ocp.set_value(obs_x, obs_x0)
ocp.set_value(obs_y, obs_y0)
ocp.set_value(obs_r, obs_r0)

p0 = vertcat(obs_x, obs_y)
p1 = vertcat(x1, y1)
p2 = vertcat(x2, y2)
p3 = vertcat(x3, y3)

ocp.subject_to(sumsqr(p1-p0) >= obs_r**2)
ocp.subject_to(sumsqr(p2-p0) >= obs_r**2)
ocp.subject_to(sumsqr(p3-p0) >= obs_r**2)

solve_func = ocp.to_function('solve_func',
	[   # Inputs
		ocp.value(x3_tf),
		ocp.value(y3_tf),
		ocp.value(theta3_tf),
		ocp.value(obs_x),
		ocp.value(obs_y),
		ocp.value(obs_r),
    ],
    [   # Outputs (optimal values)
		ocp.sample(x1,     grid='integrator', refine=refine)[0],
		ocp.sample(x1,     grid='integrator', refine=refine)[1],
		ocp.sample(y1,     grid='integrator', refine=refine)[1],
		ocp.sample(theta1, grid='integrator', refine=refine)[1],
		ocp.sample(x2,     grid='integrator', refine=refine)[1],
		ocp.sample(y2,     grid='integrator', refine=refine)[1],
		ocp.sample(theta2, grid='integrator', refine=refine)[1],
		ocp.sample(x3,     grid='integrator', refine=refine)[1],
		ocp.sample(y3,     grid='integrator', refine=refine)[1],
		ocp.sample(theta3, grid='integrator', refine=refine)[1],
		ocp.sample(v1,     grid='integrator', refine=refine)[1],
		ocp.sample(delta1, grid='integrator', refine=refine)[1],
    ],
    # define labels for input & output so these can be handled
	["x3_tf","y3_tf","theta3_tf","obs_x","obs_y","obs_r"], # Input labels
	["t","x1","y1","theta1","x2","y2","theta2","x3","y3","theta3","v1","delta1"]) # Output labels

# Solve
res = solve_func(x3_tf=-2,y3_tf=-2,theta3_tf=-3*pi/4,obs_x=obs_x0,obs_y=obs_y0,obs_r=obs_r0)

# Get solution
ts = np.array(res["t"]).squeeze()
x1 = np.array(res["x1"]).squeeze()
y1 = np.array(res["y1"]).squeeze()
theta1 = np.array(res["theta1"]).squeeze()
x2 = np.array(res["x2"]).squeeze()
y2 = np.array(res["y2"]).squeeze()
theta2 = np.array(res["theta2"]).squeeze()
x3 = np.array(res["x3"]).squeeze()
y3 = np.array(res["y3"]).squeeze()
theta3 = np.array(res["theta3"]).squeeze()
delta1 = np.array(res["delta1"]).squeeze()
v1 = np.array(res["v1"]).squeeze()

# Show results
from pylab import *

fig, ax = plt.subplots()

line1, = plt.plot(x1, y1, color='b')
line2, = plt.plot(x2, y2, color='r')
line3, = plt.plot(x3, y3, color='g')
plt.subplots_adjust(left = .35, right=0.65)

twopi = np.linspace(0,2*pi,1000)
circle, = plt.plot(obs_x0+obs_r0*cos(twopi), obs_y0+obs_r0*sin(twopi),'k-')

axamp_obs_x = plt.axes([0.7, 0.15, 0.0225, 0.7])
axamp_obs_y = plt.axes([0.8, 0.15, 0.0225, 0.7])
axamp_obs_r = plt.axes([0.9, 0.15, 0.0225, 0.7])
amp_slider_x = Slider(ax=axamp_obs_x, label="x0", valmin=-5, valmax=2, valinit=obs_x0, orientation="vertical")
amp_slider_y = Slider(ax=axamp_obs_y, label="y0", valmin=-4, valmax=0, valinit=obs_y0, orientation="vertical")
amp_slider_r = Slider(ax=axamp_obs_r, label="r0", valmin= 0, valmax=2, valinit=obs_r0, orientation="vertical")

def update(val):
	# Solve
	obs_x0 = amp_slider_x.val
	obs_y0 = amp_slider_y.val
	obs_r0 = amp_slider_r.val
	res = solve_func(obs_x=obs_x0, obs_y=obs_y0, obs_r=obs_r0)

	# Get solution
	ts = np.array(res["t"]).squeeze()
	x1 = np.array(res["x1"]).squeeze()
	y1 = np.array(res["y1"]).squeeze()
	theta1 = np.array(res["theta1"]).squeeze()
	x2 = np.array(res["x2"]).squeeze()
	y2 = np.array(res["y2"]).squeeze()
	theta2 = np.array(res["theta2"]).squeeze()
	x3 = np.array(res["x3"]).squeeze()
	y3 = np.array(res["y3"]).squeeze()
	theta3 = np.array(res["theta3"]).squeeze()
	delta1 = np.array(res["delta1"]).squeeze()
	v1 = np.array(res["v1"]).squeeze()

	line1.set_data(x1,y1)
	line2.set_data(x2,y2)
	line3.set_data(x3,y3)
	circle.set_data(obs_x0+obs_r0*cos(twopi), obs_y0+obs_r0*sin(twopi))
	fig.canvas.draw_idle()

amp_slider_x.on_changed(update)
amp_slider_y.on_changed(update)
amp_slider_r.on_changed(update)

show(block=True)
