# AMR Robot Mapping Guide

This guide provides the step-by-step process to map a room from scratch using the Autonomous Mobile Robot (AMR) workspace. 

You will need to open **four separate terminal windows** to execute this process correctly.

## Prerequisites
Ensure your workspace is built and sourced before running the launch commands.
```bash
cd ~/amr
colcon build
source install/setup.bash
```

---

## Step 1: Start the Robot Hardware
Open **Terminal 1**. This command turns on the robot's motors, the LiDAR sensor, and opens **RViz** (the visualizer window) so you can see the sensor data.
```bash
cd ~/amr
source install/setup.bash
ros2 launch amr_bringup bot.launch.py
```

## Step 2: Start the Mapping Software (SLAM)
Open **Terminal 2**. This runs `slam_toolbox`, which takes the laser data from the LiDAR and starts stitching together a map.
```bash
cd ~/amr
source install/setup.bash
ros2 launch amr_bringup online_async_launch.py
```

## Step 3: Drive the Robot Around (Teleop)
Open **Terminal 3**. To map the room, the robot needs to look around. You will use the keyboard to manually drive the robot slowly around the room.
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
*Use the `i`, `j`, `k`, `,`, `l` keys on your keyboard to drive the robot. Keep this terminal active while driving. Look at RViz on your screen; you should see the map slowly being drawn.*

## Step 4: Save the Map
Open **Terminal 4**. Once you have driven the robot all around the room and the map in RViz looks complete and enclosed, you need to save it. 

**Do not close the other terminals yet.**
```bash
cd ~/amr
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f room
```
This command tells ROS to capture the current map from memory and save it. It will generate two files in your current directory: 
- `room.pgm` (the image of the map) 
- `room.yaml` (the configuration/metadata)

Once the map is saved, you can safely press `Ctrl+C` in all the other terminals to shut the robot down.


