from truck_trailer_amr.motionplanner_multistage import MotionPlanner
from truck_trailer_amr.controller import Controller
from truck_trailer_amr.msg import Trigger_mp
import casadi as c
import numpy as np

import matplotlib.pyplot as plt
from pylab import *

param_path = "../../../config/"

# Create MotionPlanner object
print("-- Create MotionPlanner object --")
mp = MotionPlanner(standalone=True, param_path=param_path)
# Configure ocp
mp.configure()
print("-- Configured --")

# trigger_mp = Trigger_mp(u_t0=u_t0,
#                         u_tf=u_tf,
#                         x_t0=x_t0,
#                         x_tf=x_tf)

# # Preprocess
# solve_ocp_args = mp.solve_ocp_preproc(trigger_mp)

# solved with smooth reformulation on x_tf

x_t0 = [0., 0., 0., 0.]
u_t0 = [0, 0]
x_tm = [4., 0., -0.58749998, 0.]
x_tf = [3.4000001, 1.20000005, -1.37249994, -1.17499995]
u_tf = [0, 0]
W0 = np.array(
    [[1, 0, -1, 0, 1, 0, -1, 0, 1, 0, -1, 0],
     [0, 1, 0, -1, 0, 1, 0, -1, 0, 1, 0, -1],
     [-5, -0.5, -1, -0.5, -5, -1.5, 2.5, -0.5, -5, -1.5, 2.5, -0.5]])
W1 = np.array(
    [[1, 0, -1, 0, 1, 0, -1, 0, 1, 0, -1, 0],
     [0, 1, 0, -1, 0, 1, 0, -1, 0, 1, 0, -1],
     [-5, -0.5, -1, -0.5, -5, -0.5, -1, -0.5, -5, -1.5, 2.5, -0.5]])
closed_corridor_2 = 0
closed_corridor_3 = 0
direction = (0.0, 1.0)

solve_ocp_args = {'init_control': u_t0,
                  'terminal_control': u_tf,
                  'init_state': x_t0,
                  'mid_state': x_tm,
                  'terminal_state': x_tf,
                  'truck_corridor_vectors': W0,
                  'trailer_corridor_vectors': W1,
                  'closed_corridor_2': closed_corridor_2,
                  'closed_corridor_3': closed_corridor_3,
                  'direction': direction}

# Process
result_raw = mp.MPC_update(solve_ocp_args)

# Postprocess
result = mp.solve_ocp_postproc(result_raw)

plt.figure(1)
ax1 = plt.subplot(1, 1, 1)
ax1.axis('equal')
ax1.plot(result.px1_coarse, result.py1_coarse, 'o',
         color='grey', label='coarse')
ax1.plot(result.px1, result.py1, '.', color='red', label='fine')
ax1.set_xlabel('x [m]')
ax1.set_ylabel('y [m]')
ax1.legend()

plt.figure(2)
ax21 = plt.subplot(1, 3, 1)
ax21.plot(result.t_coarse, result.omega0_coarse, label='omega0_coarse')
ax21.plot(result.t_coarse, result.v0_coarse, label='v0_coarse')
ax21.set_xlabel('time [s]')
ax21.set_ylabel('omega0 [rad/s] and v0 [m/s]')
ax21.legend()

ax22 = plt.subplot(1, 3, 2)
ax22.plot(result.t_coarse, result.px1_coarse, label='px1_coarse')
ax22.plot(result.t_coarse, result.py1_coarse, label='py1_coarse')
ax22.set_xlabel('time [s]')
ax22.set_ylabel('x [m]')
ax22.legend()

ax23 = plt.subplot(1, 3, 3)
ax23.plot(result.t_coarse, result.theta1_coarse, label='theta1_coarse')
ax23.plot(result.t_coarse, result.theta0_coarse, label='theta0_coarse')
ax23.set_xlabel('time [s]')
ax23.set_ylabel('theta [rad]')
ax23.legend()

show(block=True)
