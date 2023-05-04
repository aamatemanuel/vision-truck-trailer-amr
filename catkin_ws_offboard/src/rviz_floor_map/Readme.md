# Making a ROS map and displaying it in Rviz

## 1. Create a binary figure from any png
E.g. from an image that you drew as .svg.


If the image is not binary, run `convert_to_binary.py`.
E.g. a black and white image made in Inkscape can be used directly.

## 2. Make a ROS map out of the binary png
Run `make_ROSmap.py`. It will prompt you to click two x points and two y points.
In the file, indicate what the horizontal distance in meters between the x points
and what the vertical distance in meters between the y points is, such that it
knows the scaling from pixels to meters. Give the map a name, e.g. `my_map`,
and indicate the origin (e.g. when the map is 2mx2m and you want the origin in the center, then enter 1 twice).  
Set the resolution in `make_ROSmap.py` with the `RESOLUTION` parameter. This
tells you how many pixels per meter there are in the map.


You can also give a directory in which to store the map, by default `maps`.

The result of the script is a .pgm-file and a .yaml file.

## 3. Load the ROS map in Rviz
Launch Rviz, and then run `rosrun map_server map_server my_map.yaml`.