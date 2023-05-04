#!/usr/bin/env python3
from data_handling import get_data, get_data_json
import matplotlib.pyplot as plt
import matplotlib

'''
This file loads experiment data and plots reference trajectories and
measured traveled trajectories, in space and in time.
'''

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


latexify()

t0 = data['t_p1'][0]
plt.figure(figsize=(7, 5))
plt.plot(data['setpoint_x'], data['setpoint_y'],
         linewidth=1.5, color='steelblue', label="Reference trajectory")
plt.plot(data['p1_x'], data['p1_y'],
         linewidth=1.5, color='firebrick', label='Measured positions')
plt.xlabel('x position (m)')
plt.ylabel('y position (m)')
plt.legend().set_draggable(True)


# plt.figure()
# plt.plot([val-t0 for val in data['t_comp_times']],
#          [1e3*val for val in data['comp_times']],
#          'o', color='steelblue')
# plt.xlabel('Time (s)')
# plt.ylabel('Computation time (ms)')


# plt.figure()
# plt.plot([val-t0 for val in data['t_stage_time']],
#          data['stage1_time'], color='firebrick', label='stage1.T')
# plt.plot([val-t0 for val in data['t_stage_time']],
#          data['stage2_time'], color='steelblue', label='stage2.T')
# plt.plot([val-t0 for val in data['t_stage_time']],
#          data['stage3_time'], color='xkcd:marigold', label='stage3.T')
# plt.xlabel('Time (s)')
# plt.ylabel('Stage time T (s)')
# plt.legend()


plt.savefig("figures/path.pdf", bbox_inches='tight')
# plt.show()
