# Truck-Trailer AMR Demo

## Simulation demo

- Start a roscore, the simulator, rviz, and the map server:

```
cd execution/
source run_sim
```

- Run motion planning - control nodes
```
cd execution/
roslaunch truck_trailer_amr sim_demo.launch
```
## Physical demo
### Hardware
- Charge LiPo battery
- Charge Vive Tracker
- Screw Vive Tracker on Trailer
- Screw Trailer to Angle Encoder
- Plug in Vive Lighthouses and place them around working area
    - Power
    - Connecting cable (not required but recommended)
- Plug Tracker Dongle in computer
- Turn on Tracker, light should be green
- Turn on Beamers, turn on Edge Blend Processor
- Plug HDMI in computer


<div style="page-break-after: always;"></div>

### Software
- Connect computer to BelkinG
    - pw: `brugsezot`  


- Run **VRMonitor**, Tracker and two lighthouses should be indicated
- Run *onboard components*: ssh to odroid in three terminal windows
    ```
    ssh odroid@odroid
    ```
    - pw: `odroid`
    - terminal 1: start **`roscore`**:
    ```
    rosexport
    roscore
    ```
    - terminal 2: start **`kelo_tulip`**:
    ```
    rosexport
    roslaunch kelo_tulip example.launch
    ```
    - terminal3: start **`angle_encoder_ros`**:
    ```
    rosexport
    roslaunch angle_encoder_ros encoder.launch
    ```
    (don't forget the `rosexport`, it's an alias set in odroid's .bashrc to execute `execution/rosexport_onboard`)


- Run *offboard components* in a new terminal window (Rviz, vive_localization, map_server):       
    > **Running the offboard components from a Docker container**  
    > (Skip this block if you don't want to run with Docker)   
    > - Build the Docker image (only first time):
    > ```
    > docker build . -t ttamr
    > ```
    > 
    > - Run the Docker image:
    > ```
    > ./execution/run_docker bash
    > ```
    > Running the same command without the bash starts the simulation demo. 
    > For the physical demo, follow the instructions below from within the container environment.
    ```
    cd execution/
    source run_demo_offboard
    ```
    (the ROS export is already included in run_demo_offboard)

- In Rviz select config  
`catkin_ws_offboard/src/truck_trailer_amr/config/roblab_floor_demo.rviz`
for the demo view, or
`catkin_ws_offboard/src/truck_trailer_amr/config/roblab_floor_debug.rviz`
for the debug view.

- Set correct **display resolution**
```
cd catkin_ws_offboard/src/rviz_floor_map/
./beamer_set_resolution.sh
```
or, if the correct resolution doesn't exist yet
```
cd catkin_ws_offboard/src/rviz_floor_map/
./beamer_make_resolution.sh
./beamer_set_resolution.sh
```

- Calibrate HTC Vive to match localization with visualization.
    ```
    calibrate_vive
    ```
    The terminal tab running Vive localization will display instructions to place the tracker at desired locations. When the tracker is placed at the instructed location, execute

    ```
    proceed_vive_calibration
    ```
    Wait for the point to be recorded, and repeat for all three calibration
    points.

    If you do the calibration with a separate tracker, then shut down the vive_localization node,
    turn off the tracker and turn on the tracker mounted on the vehicle, and restart
    the vive localization with
    ```
    roslaunch vive_localization localization.launch
    ```

- *(Optional)* Record experiment
    ```
    rosbag record -a
    ```
    Ctrl+C when done.