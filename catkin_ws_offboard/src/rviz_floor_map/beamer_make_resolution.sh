#!/bin/bash

#cvt 1920 2090 60

sudo xrandr --newmode "3648x2272_10.00"  104.25  3648 3744 4104 4560  2272 2275 2285 2288 -hsync +vsync
sudo xrandr --addmode DisplayPort-0 "3648x2272_10.00"
