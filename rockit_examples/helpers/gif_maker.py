"""
GIF maker
=========

Create GIF from multiple pngs
"""

import imageio

N_images = 100
basic_file_name = 'trailer'
export_name = 'movie.gif'
duration = .01

images = []
file_names = [basic_file_name+str(iter)+'.png' for iter in range(N_images)]

for file_name in file_names:
    images.append(imageio.imread(file_name))
kargs = { 'duration': duration }

imageio.mimsave(export_name, images, 'GIF', **kargs)
