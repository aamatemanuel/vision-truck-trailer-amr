# ROS Eagle Bridge

After `catkin_make` or `catkin build`, execute
```
$ rosrun eagle_ros eagles.py
```

## Calibration
From terminal:
```
rostopic pub --once /eagle_ros/calibrate std_msgs/Empty
```
or publish an Empty message on the same topic from your own ros node.
