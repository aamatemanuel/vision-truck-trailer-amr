#!/usr/bin/env python3
import numpy as np
import casadi as cs
import matplotlib.pyplot as plt
import ruamel.yaml
import io

from dlqr import dlqr

'''
Manually find stable gains for the outer edges of the control input range.
Then interpolate linearly for the v values in between.
'''
#######
# SIM #
#######

# vmin_pos = 0.1
# vmax_pos = 1.
#
# vmin_neg = -0.1
# vmax_neg = -1.

# # 0.1
# Kminv_pos = np.array([[0.05, 0., 0.],
#                       [0., 0.005, 0.01]])
# # 1.
# Kmaxv_pos = np.array([[1., 0., 0.],
#                       [0., 0.2, 0.4]])
#
# # -0.1
# Kminv_neg = np.array([[1., 0., 0.],
#                       [0., -2., 2.]])
# # -1.
# Kmaxv_neg = np.array([[1., 0., 0.],
#                       [0., -7., 7.]])

############
# Real AMR #
############
vmin_pos = 0.1
vmax_pos = 0.5

vmin_neg = -0.1
vmax_neg = -0.2


# 0.1
Kminv_pos = np.array([[0.015, 0., 0.],
                      [0., 0.010, 0.09]])
# 0.5
Kmaxv_pos = np.array([[0.15, 0., 0.],
                      [0., 0.1, 0.9]])

# -0.1
Kminv_neg = np.array([[0.7, 0., 0.],
                      [0., -0.1, 0.17]])
# -0.2
Kmaxv_neg = np.array([[0.7, 0., 0.],
                      [0., -0.2, 0.17]])


# Linear interpolation
Kcoeffspos = np.zeros((2, 3, 2))
Kcoeffsneg = np.zeros((2, 3, 2))
# Slope
Kcoeffspos[:, :, 0] = (Kmaxv_pos - Kminv_pos) / (vmax_pos - vmin_pos)
Kcoeffsneg[:, :, 0] = (Kmaxv_neg - Kminv_neg) / (vmax_neg - vmin_neg)
# Intercept
Kcoeffspos[:, :, 1] = (Kminv_pos * vmax_pos - Kmaxv_pos * vmin_pos) / (
                                                           vmax_pos - vmin_pos)
Kcoeffsneg[:, :, 1] = (Kminv_neg * vmax_neg - Kmaxv_neg * vmin_neg) / (
                                                           vmax_neg - vmin_neg)
# import pdb; pdb.set_trace()
Ni = Kminv_pos.shape[0]
Nj = Kminv_pos.shape[1]
vlpos = np.arange(vmin_pos, vmax_pos, 0.01)
vlneg = np.arange(vmax_neg, vmin_neg, 0.01)

# Plot the result
markersize = 1
ax = []
plt.figure()
ax.append(plt.subplot(231))
# ax[0].plot(vlff, K[0, 0, :], 'o', markersize=markersize)
ax[0].title.set_text('Kx')
ax[0].set_xlabel('v_ff (m/s)')
# ax[0].set_ylim([0, max(K[0, 0, :])*1.2])

ax.append(plt.subplot(232))
# ax[1].plot(vlff, K[0, 1, :], 'o', markersize=markersize)

ax.append(plt.subplot(233))
# ax[2].plot(vlff, K[0, 2, :], 'o', markersize=markersize)

ax.append(plt.subplot(234))
# ax[3].plot(vlff, K[1, 0, :], 'o', markersize=markersize)

ax.append(plt.subplot(235))
# ax[4].plot(vlff, K[1, 1, :], 'o', markersize=markersize)
ax[4].title.set_text('Ky')
ax[4].set_xlabel('v_ff (m/s)')

ax.append(plt.subplot(236))
# ax[5].plot(vlff, K[1, 2, :], 'o', markersize=markersize)
ax[5].title.set_text('Ktheta')
ax[5].set_xlabel('v_ff (m/s)')

plt.subplots_adjust(hspace=0.5, wspace=0.8)

for i in range(Ni):
    for j in range(Nj):
        fitneg = Kcoeffsneg[i, j, :] @ np.concatenate(
                                              [[vlneg], [np.ones(vlneg.size)]])
        fitpos = Kcoeffspos[i, j, :] @ np.concatenate(
                                              [[vlpos], [np.ones(vlpos.size)]])
        ax[j + i*Nj].plot(vlneg, fitneg)
        ax[j + i*Nj].plot(vlpos, fitpos)

plt.show()


# Store the result
Kfb = {'Kfb_coeffs_pos': Kcoeffspos.tolist(),
       'Kfb_coeffs_neg': Kcoeffsneg.tolist()}
yaml = ruamel.yaml.YAML()
with io.open('Kfb.yaml', 'w') as outfile:
    yaml.dump(Kfb, outfile)
