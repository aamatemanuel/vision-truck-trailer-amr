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

Simple motion planning with circular obstacle
"""

from rockit import *
import matplotlib.pyplot as plt
import numpy as np
from numpy import pi, cos, sin, tan
from casadi import vertcat, sumsqr
from plot_trailer import *
import yaml

ocp = Ocp(T=FreeTime(20.0))
N = 25
M = 2
refine = 2

# Environment
og = 0
sg = sin(og)
cg = cos(og)

xcr = 0.5
ycr = 2.
pcr = vertcat(xcr, ycr)

xcl = -0.5
ycl = 2.3
pcl = vertcat(xcl, ycl)

n1_1 = vertcat(-cg, sg)
w1_1 = vertcat(n1_1, -n1_1.T @ pcl)

n2_1 = -n1_1
w2_1 = vertcat(n2_1, -n2_1.T @ pcr)

n1_2 = vertcat(sg, cg)
w1_2 = vertcat(n1_2, -n1_2.T @ pcl)

n2_2 = -n1_2
w2_2 = vertcat(n2_2, -n2_2.T @ pcr)

# Parameters
with open('truck_trailer_para.yaml', 'r') as file:
    para = yaml.safe_load(file)

L0 = para['truck']['L']
M0 = para['truck']['M']
W0 = para['truck']['W']
L1 = para['trailer1']['L']
M1 = para['trailer1']['M']
W1 = para['trailer1']['W']

x1_t0 = 0.
y1_t0 = 0.
theta1_t0 = pi/2
theta0_t0 = pi/2

x1_tf = 1.5
y1_tf = (ycl + ycr)/2
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
ocp.set_initial(theta0, 0.)
ocp.set_initial(theta1, 0.)
ocp.set_initial(v0,     0.)
ocp.set_initial(x1,     np.linspace(x1_t0,x1_tf,N))
ocp.set_initial(y1,     np.linspace(y1_t0,y1_tf,N))
 
# Path constraints
ocp.subject_to(-.2 <= (v0 <= .2))
ocp.subject_to(-1 <= (ocp.der(v0) <= 1))

ocp.subject_to(-pi/6 <= (delta0 <= pi/6))
ocp.subject_to(-pi/10 <= (ocp.der(delta0) <= pi/10))

ocp.subject_to(-pi/2 <= (beta01 <= pi/2))

# Obstacle avoidance with SH
SH_ax = ocp.control(order=0)
SH_ay = ocp.control(order=0)
SH_b  = ocp.control(order=0)

ocp.subject_to(SH_ax**2 + SH_ay**2 == 1)

veh_vertices = vert_vehic(x0, y0, theta0, W0/2, W0/2, L0, M0)
for veh_vertex in veh_vertices:
	ocp.subject_to(SH_ax*veh_vertex[0] + SH_ay*veh_vertex[1] >= SH_b, grid='integrator')

veh_vertices = vert_vehic(x1, y1, theta1, W1/2, W1/2, L1, M1)
for veh_vertex in veh_vertices:
	ocp.subject_to(SH_ax*veh_vertex[0] + SH_ay*veh_vertex[1] >= SH_b, grid='integrator')
	
obst_vertices = [[.5, -1.], [.5, 2.], [4., 2.], [4., -1.]]
obst_vertices_x = [vertices[0] for vertices in obst_vertices]
obst_vertices_y = [vertices[1] for vertices in obst_vertices]
for obst_vertex_x, obst_vertex_y in zip(obst_vertices_x, obst_vertices_y):
    ocp.subject_to(SH_ax*obst_vertex_x + SH_ay*obst_vertex_y <= SH_b)

# Obstacle avoidance with SH2
SH2_ax = ocp.control(order=0)
SH2_ay = ocp.control(order=0)
SH2_b  = ocp.control(order=0)

ocp.subject_to(SH2_ax**2 + SH2_ay**2 == 1)
veh_vertices = vert_vehic(x0, y0, theta0, W0/2, W0/2, L0, M0)
for veh_vertex in veh_vertices:
	ocp.subject_to(SH2_ax*veh_vertex[0] + SH2_ay*veh_vertex[1] >= SH2_b, grid='integrator')

veh_vertices = vert_vehic(x1, y1, theta1, W1/2, W1/2, L1, M1)
for veh_vertex in veh_vertices:
	ocp.subject_to(SH2_ax*veh_vertex[0] + SH2_ay*veh_vertex[1] >= SH2_b, grid='integrator')

obst_vertices = [[-3, ycl], [-3., ycl+1], [3., ycl+1], [3., ycl]]
obst_vertices_x = [vertices[0] for vertices in obst_vertices]
obst_vertices_y = [vertices[1] for vertices in obst_vertices]
for obst_vertex_x, obst_vertex_y in zip(obst_vertices_x, obst_vertices_y):
    ocp.subject_to(SH2_ax*obst_vertex_x + SH2_ay*obst_vertex_y <= SH2_b)

# Obstacle avoidance with SH3
SH3_ax = ocp.control(order=0)
SH3_ay = ocp.control(order=0)
SH3_b  = ocp.control(order=0)

ocp.subject_to(SH3_ax**2 + SH3_ay**2 == 1)
veh_vertices = vert_vehic(x0, y0, theta0, W0/2, W0/2, L0, M0)
for veh_vertex in veh_vertices:
	ocp.subject_to(SH3_ax*veh_vertex[0] + SH3_ay*veh_vertex[1] >= SH3_b, grid='integrator')

veh_vertices = vert_vehic(x1, y1, theta1, W1/2, W1/2, L1, M1)
for veh_vertex in veh_vertices:
	ocp.subject_to(SH3_ax*veh_vertex[0] + SH3_ay*veh_vertex[1] >= SH3_b, grid='integrator')

obst_vertices = [[-3, -3], [-3., 3], [xcl, 3], [xcl, -3]]
obst_vertices_x = [vertices[0] for vertices in obst_vertices]
obst_vertices_y = [vertices[1] for vertices in obst_vertices]
for obst_vertex_x, obst_vertex_y in zip(obst_vertices_x, obst_vertices_y):
    ocp.subject_to(SH3_ax*obst_vertex_x + SH3_ay*obst_vertex_y <= SH3_b)

# Minimal time
ocp.add_objective(ocp.T)
#ocp.add_objective(ocp.integral(beta**2))

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

# Solve
sol = ocp.solve()

# Get solution
ts, theta1 = sol.sample(theta1, grid='integrator', refine=refine)
ts, x1     = sol.sample(x1,     grid='integrator', refine=refine)
ts, y1     = sol.sample(y1,     grid='integrator', refine=refine)

ts, theta0 = sol.sample(theta0, grid='integrator', refine=refine)
ts, x0     = sol.sample(x0,     grid='integrator', refine=refine)
ts, y0     = sol.sample(y0,     grid='integrator', refine=refine)

ts, delta0 = sol.sample(delta0, grid='integrator', refine=refine)
ts, v0     = sol.sample(v0,     grid='integrator', refine=refine)

ts, SH_ax  = sol.sample(SH_ax,  grid='integrator', refine=refine)
ts, SH_ay  = sol.sample(SH_ay,  grid='integrator', refine=refine)
ts, SH_b   = sol.sample(SH_b,   grid='integrator', refine=refine)

# Show results
from pylab import *

plt.figure(1)
ax1 = plt.subplot(1, 1, 1)
ax1.axis('equal')

ax1.plot(x0, y0, color='k')
ax1.plot(x1, y1, color='r')

draw_constraint(w1_1.full().T[0], ax1, 'red')
draw_constraint(w2_1.full().T[0], ax1, 'red')
draw_constraint(w1_2.full().T[0], ax1, 'red')
draw_constraint(w2_2.full().T[0], ax1, 'red')
ax1.set_ylim(-2, 4)

for k in range(N*M*refine + 1):
	x0s     = x0[k]
	y0s     = y0[k]
	theta0s = theta0[k]
	x1s     = x1[k]
	y1s     = y1[k]
	theta1s = theta1[k]
	delta0s = delta0[k]

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

	#png_name = 'trailer'+str(k)+'.png'
	#plt.savefig(png_name)

	if k == 0:
		pause(2.)

	pause(.01)
	if k < N*M*refine:
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
