#!/usr/bin/env python3
import numpy as np
import casadi as cs
import matplotlib.pyplot as plt
import ruamel.yaml
import io

from dlqr import dlqr

Ts = 0.04
tol = 1e-9

'''
x = [px, py, pz]'
u = [vl, omega]'
'''
x = cs.MX.sym('x', 3)
u = cs.MX.sym('u', 2)

f = cs.Function('f', [x, u], [cs.vertcat(u[0] * cs.cos(x[2]),
                                         u[0] * cs.sin(x[2]),
                                         u[1])])

dfdx = cs.Function('dfdx', [x, u], [cs.jacobian(f(x, u), x)])
dfdu = cs.Function('dfdu', [x, u], [cs.jacobian(f(x, u), u)])

# Discrete dynamics
A = cs.Function('A', [x, u], [np.eye(3) + dfdx(x, u) * Ts])
B = cs.Function('B', [x, u], [dfdu(x, u) * Ts])
F = cs.Function('F', [x, u], [A(x, u) @ x + B(x, u) @ u])


# Compute dlqr for a range of u values
Q = np.diag([1e0, 3e0, 3e0])
R = 1e0*np.diag([1e0, 1e0])


'''
Creating a 'deadzone' around vl_ff = 0, because for that value no finite gain
can be found that drives the y-error to 0. (you can rotate, but there's no
velocity component driving the position error to 0)
'''
vlneg = np.linspace(-3, -0.1, 100)
vlpos = np.linspace(0.1, 3., 100)

vlff = np.concatenate((vlneg, vlpos))
K = np.zeros((2, 3, len(vlff)))

for i, vl in enumerate(vlff):
    # print(i, vl)
    xi = cs.vertcat(0, 0, 0)  # evaluating in equilibrium
    ui = cs.vertcat(vl, 0)  # omega value has no influence on jacobians

    Ai = A(xi, ui).full()
    Bi = B(xi, ui).full()

    Ki = dlqr(Ai, Bi, Q, R)
    Ki[abs(Ki) < tol] = 0
    K[:, :, i] = Ki

# Plot the result
plt.figure()
plt.plot(vlff)
plt.title('vl feedforward value')

markersize = 1
ax = []
plt.figure()
ax.append(plt.subplot(231))
ax[0].plot(vlff, K[0, 0, :], 'o', markersize=markersize)
ax[0].title.set_text('Kx')
ax[0].set_xlabel('v_ff (m/s)')
ax[0].set_ylim([0, max(K[0, 0, :])*1.2])

ax.append(plt.subplot(232))
ax[1].plot(vlff, K[0, 1, :], 'o', markersize=markersize)

ax.append(plt.subplot(233))
ax[2].plot(vlff, K[0, 2, :], 'o', markersize=markersize)

ax.append(plt.subplot(234))
ax[3].plot(vlff, K[1, 0, :], 'o', markersize=markersize)

ax.append(plt.subplot(235))
ax[4].plot(vlff, K[1, 1, :], 'o', markersize=markersize)
ax[4].title.set_text('Ky')
ax[4].set_xlabel('v_ff (m/s)')

ax.append(plt.subplot(236))
ax[5].plot(vlff, K[1, 2, :], 'o', markersize=markersize)
ax[5].title.set_text('Ktheta')
ax[5].set_xlabel('v_ff (m/s)')

plt.subplots_adjust(hspace=0.5, wspace=0.8)

# Make a linear fit through the elements of K
Ni = K.shape[0]
Nj = K.shape[1]
Kcoeffsneg = np.zeros((Ni, Nj, 2))
Kcoeffspos = np.zeros((Ni, Nj, 2))
for i in range(Ni):
    for j in range(Nj):
        Kcoeffsneg[i, j, :] = np.polyfit(vlneg, K[i, j, 0:len(vlneg)], 1)
        Kcoeffspos[i, j, :] = np.polyfit(vlpos, K[i, j, len(vlneg):], 1)
        Kcoeffsneg[abs(Kcoeffsneg) < tol] = 0
        Kcoeffspos[abs(Kcoeffspos) < tol] = 0
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
