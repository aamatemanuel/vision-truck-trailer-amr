#!/usr/bin/env python3
from data_handling import get_data, get_data_json, find_data_in_trange
import matplotlib
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import numpy as np

data_directory = '../recorded_bags/paper_experiment/'
data_name = 'run_full_filmed_2022-11-07-16-40-56'
extension = '.json'  # '.bag'  #
datafile = data_directory + data_name + extension

# Get data
if extension == '.bag':
    data = get_data(datafile)
elif extension == '.json':
    data = get_data_json(datafile)


# Plotting
def latexify():
    params = {'backend': 'ps',
              'axes.labelsize': 25,
              'axes.titlesize': 20,
              'legend.fontsize': 20,
              'xtick.labelsize': 25,
              'ytick.labelsize': 25,
              'text.usetex': True,
              'font.family': 'serif',
              'text.latex.preamble': r"\usepackage{bm}"
              }

    matplotlib.rcParams.update(params)


# Select data range in time
t0 = data['t_setpoint'][0]
t_p1 = [t-t0 for t in data['t_p1']]
t_sp = [t-t0 for t in data['t_setpoint']]
tstart = 0.
tend = 88.75

# Select time samples between tstart and tend
t, p1_x = find_data_in_trange(t_p1, data['p1_x'], tstart, tend)

# Interpolate data to be on same timings
f_p1y = interp1d(t_p1, data['p1_y'],
                 fill_value='extrapolate')
f_x = interp1d(t_sp, data['setpoint_x'],
               fill_value='extrapolate')
f_y = interp1d(t_sp, data['setpoint_y'],
               fill_value='extrapolate')

p1_y = f_p1y(t).tolist()
setpoint_x_interp = f_x(t).tolist()
setpoint_y_interp = f_y(t).tolist()

error_norm = [np.sqrt(
    (p1_x[i] - setpoint_x_interp[i])**2 +
    (p1_y[i] - setpoint_y_interp[i])**2)
    for i in range(len(t))]

latexify()

plt.figure()
plt.plot(data['setpoint_x'], data['setpoint_y'], 'o',
         color='steelblue', label='Trailer setpoint')
plt.plot(data['p1_x'], data['p1_y'], 'x',
         color='firebrick', label='Trailer position')


plt.figure()
plt.plot(t_sp, data['setpoint_x'], 'o',
         color='steelblue', label='Trailer setpoint x')
plt.plot(t_sp, data['setpoint_y'], 'o',
         color='steelblue', label='Trailer setpoint y')
plt.plot(t_p1, data['p1_x'], 'x',
         color='firebrick', label='Trailer x position')
plt.plot(t_p1, data['p1_y'], 'x',
         color='firebrick', label='Trailer y position')
plt.plot(t, setpoint_x_interp, 'd', label='Interpolated x setpoint')
plt.plot(t, setpoint_y_interp, '^', label='Interpolated y setpoint')

plt.legend().set_draggable(True)

plt.figure()
plt.plot([t-t0 for t in data['t_setpoint']], len(data['t_setpoint'])*[1],
         'o', color='steelblue', label='setpoint time')
plt.plot([t-t0 for t in data['t_p1']], len(data['t_p1'])*[1],
         'x', color='firebrick', label='measured pos time')
plt.legend()

plt.figure(figsize=(7, 5.15))
plt.plot(t,
         [1e2*val for val in error_norm],
         color='xkcd:marigold')
# plt.plot(error_norm[rng], color='xkcd:marigold')
plt.xlabel('Time (s)')
plt.ylabel(r'$||\delta \bm{p}||_{2}$ (cm)')


plt.savefig("figures/tracking_error.pdf", bbox_inches='tight')
plt.show()
