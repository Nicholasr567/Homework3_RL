# Robotics Lab 2025 -- Homework 3


**Arena Giuseppe - Ruggiero Nicholas**

The goal is to build and simulate a customized multi-rotor UAV in
**PX4**, modify the **force land** node, and design a **trajectory
planner in offboard mode**.

## Prerequisites

-   Docker installed

-   Repository cloned (this repo must contain PX4-Autopilot, ROS2
    workspace, and the packages `force_land` and `offboard_rl`)

-   X11 graphical access from inside the container

## Docker Container Setup

### 1. Build the image

From inside the main project folder (`aerial_robotics-master` or
equivalent):

``` bash
docker build -t aerial_lab_image -f Dockerfile/dockerFile_RL_aerialRobotics_humble_harmonic .
```

### 2. Enable graphical access

On the host system (outside Docker):

``` bash
xhost +local:root
```

### 3. Start the container

From inside `aerial_robotics-master`:

``` bash
docker run -it --rm     --name aerial_container     --net=host     --privileged     --env="DISPLAY=$DISPLAY"     --env="QT_X11_NO_MITSHM=1"     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"     --volume="$(pwd):/root/aerial_robotics-master"     aerial_lab_image
```

To re-enter an already running container (without recreating it):

``` bash
docker exec -it aerial_container bash
```

### 4. PX4/ROS2 environment setup

Inside the container, for the first time (and whenever required):

``` bash
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

Then go back to root and run:

``` bash
cd /root/aerial_robotics-master
source /root/aerial_robotics-master/setup_env.sh
```

Alternatively, you can add the configurations directly to the
container's `~/.bashrc` to avoid reconfiguring every time:

``` bash
echo "
# --- Robotics Lab Config ---
source /opt/ros/humble/setup.bash
source /root/aerial_robotics-master/ros2_ws/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export GZ_SIM_RESOURCE_PATH=/root/PX4-Autopilot/Tools/simulation/gz/models
export PX4_SOURCE_DIR=/root/aerial_robotics-master/PX4-Autopilot
# ---------------------------
" >> ~/.bashrc
```

------------------------------------------------------------------------

## How to launch your drone (point 1)

Inside the container (after environment setup):

### 1. Start PX4 + Gazebo simulation

``` bash
cd /root/aerial_robotics-master/PX4-Autopilot
make px4_sitl gz_my_x500
```

When you see:

``` text
INFO  [gz_bridge] world: default, model: my_x500_0
```

it means the model was loaded successfully.

### 2. Required terminals

-   **Terminal 1**: PX4 + Gazebo (already running with
    `make px4_sitl gz_my_x500`)

-   **Terminal 2**:

    ``` bash
    MicroXRCEAgent udp4 -p 8888
    ```

-   **Terminal 3**:

    ``` bash
    ros2 topic list | grep actuator
    ros2 run plotjuggler plotjuggler
    ```

Then, in **Terminal 1**:

``` bash
commander arm -f
commander takeoff
# flight...
commander land
```

In **Terminal 3**, use PlotJuggler to plot
`out/actuator_outputs/command[0]` and `command[1]`.

------------------------------------------------------------------------

## Point 2 -- Testing the *force_land* node

### Commands for simulation

Inside the container, launch the terminals as follows:

-   **Terminal 1**:

    ``` bash
    cd /root/aerial_robotics-master/PX4-Autopilot
    make px4_sitl gz_my_x500
    ```

-   **Terminal 2**:

    ``` bash
    MicroXRCEAgent udp4 -p 8888
    ```

-   **Terminal 3**:

    ``` bash
    ros2 run force_land force_land
    ```

-   **Terminal 4**:

    ``` bash
    ros2 run offboard_rl go_to_point
    ```

-   **Terminal 5 (data recording)**:

    ``` bash
    ros2 bag record -o force_land_test     /fmu/out/vehicle_local_position_v1     /fmu/in/trajectory_setpoint
    ```

Procedure:

1.  In **Terminal 1**:

    ``` bash
    commander arm -f
    ```

2.  Start **Terminal 5** (rosbag).\

3.  Start the nodes in **Terminal 3** and **Terminal 4**.\

4.  In **Terminal 4**, enter the desired position:

    ``` text
    0 0 25 0 10
    ```

    (go to \[x=0, y=0, z=25, yaw=0\] in 10 seconds).

5.  When `LAND` appears in **Terminal 3**, go back to **Terminal 4** and
    insert:

    ``` text
    0 0 25 0 5
    ```

    to send a new command and show that force land is not triggered again if the pilot has regained control.

## Point 3 -- Trajectory planner (*offboard_rl*)

For the trajectory planner, you will use the `go_to_point` node (package
`offboard_rl`).

### Typical setup

-   **Terminal 1**:

    ``` bash
    cd /root/aerial_robotics-master/PX4-Autopilot
    make px4_sitl gz_my_x500
    ```

-   **Terminal 2**:

    ``` bash
    MicroXRCEAgent udp4 -p 8888
    ```

-   **Terminal 3**:

    ``` bash
    ros2 run offboard_rl go_to_point
    # inside the program select mode 2 (multi-waypoint mode)
    ```

-   **Terminal 4 (logging)**:

    ``` bash
    ros2 bag record -o trajectory_test     /fmu/out/vehicle_local_position_v1     /fmu/in/trajectory_setpoint
    ```

In **Terminal 1**:

``` bash
commander arm -f
```

Then press Enter in **Terminal 3** to start the trajectory.\
Make sure the trajectory:

-   has at least 7 waypoints;

-   does not stop the drone at intermediate waypoints (only at the end velocity should be 0 m/s).


## Modified PX4 Files for This Homework

The **PX4-Autopilot** repository included in this project has been modified to bypass issues that prevented its correct integration into the workspace.  
Below is the list of file paths and custom additions required for the completion of this Homework:

-   A new airframe `4002_gz_my_x500` was added to:

    ```
    aerial_robotics-master/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
    ```

-   A custom Gazebo model for `my_x500` was added (containing `model.sdf` and `model.config`) in:

    ```
    aerial_robotics-master/PX4-Autopilot/Tools/simulation/gz/models/
    ```

-   The main PX4 build configuration file `CMakeLists.txt` was modified.
    ```
    aerial_robotics-master/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
    ```
-   An additional modification was made to the DDS interface definition file:

    ```
    aerial_robotics-master/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
    ```

