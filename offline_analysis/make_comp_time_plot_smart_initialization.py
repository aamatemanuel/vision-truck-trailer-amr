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

# Get data without corridor closing
data_name = 'bad_initialization_failed_at_second_part'
datafile = data_directory + data_name + extension

data_noinit = get_data_json(datafile)


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

markersize = 5
# marker_val = 1250
plt.figure(figsize=(6, 4))
# With smart initialization
t0 = data_class['t_comp_times'][0]
plt.plot([t-t0 for t in data_class['t_comp_times']],
         [1e3*val for val in data_class['comp_times']],
         '.', markersize=markersize,
         color='firebrick', label='Smart initialization')
# plt.plot([t-t0 for t in data_class['t_corridor_change']],
#          # data_class['corridor_change'],
#          [marker_val]*len(data_class['corridor_change']),
#          'd', markersize=10, color='steelblue',
#          label='Corridor toggle')
# Without smart initialization
t0 = data_noinit['t_comp_times'][0]
plt.plot([t-t0 for t in data_noinit['t_comp_times']],
         [1e3*val for val in data_noinit['comp_times']],
         '.', markersize=markersize,
         color='steelblue', label='No smart initialization')
# plt.plot([t-t0 for t in data_noinit['t_corridor_change']],
#          # data_noinit['corridor_change'],
#          [marker_val]*len(data_noinit['corridor_change']),
#          'd', markersize=10, color='firebrick',
#          label='Corridor toggle')

# plt.legend().set_draggable(True)

handles, labels = plt.gca().get_legend_handles_labels()

# specify order of items in legend
order = [0, 1]

# add legend to plot
plt.legend([handles[idx] for idx in order], [labels[idx] for idx in order])


plt.xlabel('Time (s)')
plt.ylabel('Computation time (ms)')


plt.savefig("figures/comp_times.pdf", bbox_inches='tight')
plt.show()
