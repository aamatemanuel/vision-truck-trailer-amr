#!/usr/bin/env python3
import numpy as np
import cv2
import math
import os.path
import os

'''
Based on the code found here:
https://automaticaddison.com/how-to-create-a-map-for-ros-from-a-floor-plan-or-blueprint/
'''


# =============================================================================
# FILL IN YOUR PARAMETERS HERE

file_name = 'maps_src/road4.png'
map_name = 'road4'

map_location = os.getcwd() + '/maps'
RESOLUTION = 0.001
negative = False

xdist = 6  # The horizontal distance between the first two clicked points
ydist = 3.5  # The vertical distance between the second two clicked points
origin = [1., 0.5]  # The origin of the map in meters

# =============================================================================

# -----------------
# Read in the image
# -----------------
image = cv2.imread(file_name)

if not image.any():
    print("Image '" + file_name + "' not found")
    exit()
# --------------
# Some variables
# --------------
ix, iy = -1, -1
x1 = [-1, -1, -1, -1]
y1 = [-1, -1, -1, -1]
font = cv2.FONT_HERSHEY_SIMPLEX

# ---------------------------------
# mouse callback function
# This allows to point and
# it prompts from the command line
# ---------------------------------

prompt = '> '
print("Double Click the first x point to scale")


def draw_point(event, x, y, flags, param):
    global ix, iy, x1, y1n, sx, sy
    if event == cv2.EVENT_LBUTTONDBLCLK:
        ix, iy = x, y
        print(ix, iy)

    # ------------------------------------------------------
    # Draws the point with lines around it so you can see it
    # ------------------------------------------------------
        # image[iy, ix] = (0, 0, 255)
        # cv2.line(image, (ix+2, iy), (ix+10, iy), (0, 0, 255), 1)
        # cv2.line(image, (ix-2, iy), (ix-10, iy), (0, 0, 255), 1)
        # cv2.line(image, (ix, iy+2), (ix, iy+10), (0, 0, 255), 1)
        # cv2.line(image, (ix, iy-2), (ix, iy-10), (0, 0, 255), 1)
    # ------------------------------------------------------
    # This is for the 4 mouse clicks and the x and y lengths
    # ------------------------------------------------------
        if x1[0] == -1:
            x1[0] = 0
            y1[0] = 0
            print('Double click a second x point')
        elif (x1[0] != -1 and x1[1] == -1):
            x1[1] = 567
            y1[1] = 0
            prompt = '> '
            # print("What is the x distance in meters between the 2 points?")
            deltax = xdist  # float(input(prompt))
            dx = math.sqrt((x1[1]-x1[0])**2 + (y1[1]-y1[0])**2)*RESOLUTION
            sx = deltax / dx
            print('Double Click a y point')
        elif (x1[1] != -1 and x1[2] == -1):
            x1[2] = 0
            y1[2] = 0
            print('Double click a second y point')
        else:
            prompt = '> '
            # print("What is the y distance in meters between the 2 points?")
            deltay = ydist  # float(input(prompt))
            x1[3] = 0
            y1[3] = 331
            dy = math.sqrt((x1[3]-x1[2])**2 + (y1[3]-y1[2])**2)*RESOLUTION
            sy = deltay/dy
            print(sx, sy)
            res = cv2.resize(image, None, fx=sx, fy=sy,
                             interpolation=cv2.INTER_CUBIC)
            # Convert to grey
            res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

            cv2.imshow("Residual", res)
            # Show the image in a new window
            #  Open a file
            prompt = '> '
            mapName = map_name

            prompt = '> '
            mapLocation = map_location
            completeFileNameMap = os.path.join(mapLocation, mapName + ".pgm")
            completeFileNameYaml = os.path.join(mapLocation, mapName + ".yaml")
            yaml = open(completeFileNameYaml, "w")
            cv2.imwrite(completeFileNameMap, res)

            # print('Define origin on the map (in meter)')
            origin_x = origin[0]  # float(input('x: '))
            origin_y = origin[1]  # float(input('y: '))

            # ------------------------------------
            # Write some information into the file
            # ------------------------------------
            yaml.write("image: " + mapLocation + "/" + mapName + ".pgm\n")
            yaml.write("resolution: " + str(RESOLUTION) + "\n")
            yaml.write("origin: [" + str(-origin_x) + "," + str(-origin_y) +
                       ", 0.000000]\n")
            yaml.write("negate: " + str(int(negative)) +
                       "\noccupied_thresh: 0.65\nfree_thresh: 0.196")
            yaml.close()
            exit()


cv2.namedWindow('image', cv2.WINDOW_NORMAL)
cv2.setMouseCallback('image', draw_point)
#
#  Waiting for a Esc hit to quit and close everything
#
while True:
    cv2.imshow('image', image)
    k = cv2.waitKey(20) & 0xFF
    if k == 27:
        break
    elif k == ord('a'):
        print('Done')
cv2.destroyAllWindows()
