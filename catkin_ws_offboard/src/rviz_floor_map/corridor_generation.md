# Automatically generating corridors from a map

Launch `map.launch` with the desired map, launch rviz and run the corridor
generator::

    roslaunch rviz_floor_map map.launch
    rviz
    rosrun rviz_floor_map corridor_generator.py

Click the four corners of the first corridor in clockwise order, using the Publish Point
tool. Next, click the location of the waypoint associated with the corridor, and a next
point in the direction of the desired orientation related to that waypoint.

To start with a new corridor within the same subroute, press 0 and repeat. 
To start a new subroute, press 1, and repeat.
To stop, press Ctrl+C. Every time you complete a corridor, the .yaml file is saved,
so Ctrl+C will not cause data loss.