#!/usr/bin/env python3
from data_handling import get_data, get_data_json
import matplotlib.pyplot as plt
import matplotlib
import numpy as np

# Get data classic
data_directory = '../recorded_bags/paper_simulation/'
data_name = 'classic'
extension = '.json'  # '.bag'  #
datafile = data_directory + data_name + extension

data_class = get_data_json(datafile)


# Plotting
def latexify():
    params = {'backend': 'ps',
              'axes.labelsize': 20,
              'axes.titlesize': 20,
              'legend.fontsize': 17,
              'xtick.labelsize': 20,
              'ytick.labelsize': 20,
              'text.usetex': True,
              'font.family': 'serif',
              'text.latex.preamble': r"\usepackage{bm}"
              }

    matplotlib.rcParams.update(params)


latexify()

sl = slice(0, int(400))
markersize = 5
# marker_val = 1250
plt.figure(figsize=(8, 6))
ax = plt.subplot(111)
# With smart initialization
t0 = data_class['t_comp_times'][0]

plt.plot([t-t0 for t in data_class['t_comp_times'][sl]],
         [1e3*max(data_class['comp_times'][sl])
          for _ in data_class['comp_times'][sl]],
         label='Findeisen update time', color='steelblue', linewidth=3)
plt.plot([t-t0 for t in data_class['t_comp_times'][sl]],
         [20 for _ in data_class['comp_times'][sl]],
         label='Full rate MPC update time', color='firebrick', linewidth=3)
plt.plot([t-t0 for t in data_class['t_comp_times'][sl]],
         [1e3*val for val in data_class['comp_times'][sl]],
         'o', markersize=1.5*markersize,  # markerfacecolor='none',
         color='xkcd:marigold', label='ASAP MPC update time')
plt.plot([t-t0 for t in data_class['t_comp_times'][sl]],
         [1e3*val for val in data_class['comp_times'][sl]],
         '.', markersize=markersize,
         color='black', label='Computation times')

# plt.plot([1e3*val for val in data_class['comp_times']])

plt.legend().set_draggable(True)

ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

plt.xlabel('Time (s)')
plt.ylabel('Computation time (ms)')


plt.savefig("figures/ASAP_Find_MPC_comp_times.pdf", bbox_inches='tight')
plt.show()
