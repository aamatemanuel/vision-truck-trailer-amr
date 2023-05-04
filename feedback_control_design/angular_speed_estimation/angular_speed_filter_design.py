#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import bagpy
from bagpy import bagreader
import pandas as pd
from scipy.signal import butter, lfilter, tf2ss
from speed_estimator import SpeedEstimator


# Get data
# --------
b = bagreader('truck_trailer_openloop_forward2022-05-04-13-31-37.bag')

angle_encoder_msg = b.message_by_topic('/angle_encoder/angle_meas')
angle_encoder_data = pd.read_csv(angle_encoder_msg)

t = angle_encoder_data.values[:, 0]
t -= t[0]
beta01_meas = angle_encoder_data.values[:, 1]

dt = np.median(np.gradient(t))
# plt.figure()
# plt.plot(dt)
# plt.show()


# Filter design
# -------------
Nbutt = 2
fn = 1  # For digital filter, wn has the same units as fs

b, a = butter(Nbutt, fn, fs=1/dt)

# Finite difference + Filter
dbeta01_FD = np.gradient(beta01_meas) / dt
dbeta01_FD_filt = lfilter(b, a, dbeta01_FD)

# First filter, then FD --> for fine sampling don't really see a difference,
# but is not identical though (bec gradient uses central 2nd orderd diffs.)
beta01_filt = lfilter(b, a, beta01_meas)
dbeta01_filt_FD = np.gradient(beta01_filt) / dt

# 'Manual' filtering in for loop
A, B, C, D = tf2ss(b, a)
N = len(beta01_meas)
x = np.zeros((A.shape[1], N))
y = np.zeros((1, N))
y[:, 0] = dbeta01_FD[0]
u = np.array([dbeta01_FD])
for i in range(2, N):
    x[:, i] = A @ x[:, i-1] + B @ u[:, i-1]
    y[:, i] = C @ x[:, i] + D @ u[:, i]


# Use the SpeedEstimator class
# ----------------------------
params = {'sample_time': dt, 'butter_order': Nbutt, 'butter_cutoff': fn}
speed_estimator = SpeedEstimator(params)

xclass = np.zeros((A.shape[1], N))
yclass = np.zeros((1, N))
yclass[:, 0] = dbeta01_FD[0]
for i in range(2, N):
    yclass[:, i] = speed_estimator.estimate_vel(beta01_meas[i])

# Plots
# -----
plt.figure()
ax = plt.subplot(211)
ax.plot(t, beta01_meas)
ax.set_xlabel("Time (s)")
ax.set_ylabel(r"$\beta_{01}$ (rad)")

ax = plt.subplot(212)
ax.plot(t, dbeta01_FD, 'o', color='steelblue', markersize=2,
        label="Finite difference")
ax.plot(t, dbeta01_FD_filt, color="firebrick", label="FD - Filtered")
ax.plot(t, dbeta01_filt_FD, color="gold", label="Filtered - FD")
ax.plot(t, y[0], color="green", label="Filtered - FD - manual loop")
ax.plot(t, yclass[0], color="orange", label="SpeedEStimator class")
# Difference between yclass and the others can be explained by the different
# order of Finite Differences (np.gradient = 2nd order central, class = 1st
# order Forward Euler)

ax.set_xlabel("Time (s)")
ax.set_ylabel(r"$\frac{d\beta_{01}}{dt}$ (rad/s)")
ax.legend()
plt.subplots_adjust(hspace=0.5, wspace=0.8)

plt.show()
