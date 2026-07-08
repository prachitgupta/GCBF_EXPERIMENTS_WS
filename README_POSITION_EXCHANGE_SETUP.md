# Vicon 8-Crazyflie Position Exchange Setup

This is the root-level copy of the GCBF+ Crazyflie position-exchange runbook.
The package-local copy lives at:

```text
src/crazyflie_vicon_bringup/README_POSITION_EXCHANGE_SETUP.md
```

## What This Uses

- ROS 2 Humble
- Crazyswarm2
- `motion_capture_tracking`
- Vicon host `10.192.46.131`
- `cmdFullState` streaming through `crazyflie_py`

This path does not use `vicon_bridge`, Jackal, Clearpath, rendezvous scripts, or
the old safety heartbeat nodes.

## Build

```bash
source /opt/ros/humble/setup.bash
cd /home/prachit/gcbf_experiments_ws
colcon build --symlink-install
source install/local_setup.bash
```

## Configure Crazyflies

Scan powered Crazyflies:

```bash
source /opt/ros/humble/setup.bash
source /home/prachit/gcbf_experiments_ws/install/local_setup.bash
ros2 run crazyflie scan
```

Update these files as needed:

- `src/crazyflie_vicon_bringup/config/crazyflies_single.yaml`
- `src/crazyflie_vicon_bringup/config/crazyflies_3.yaml`
- `src/crazyflie_vicon_bringup/config/crazyflies_8.yaml`

Every robot key must exactly match the Vicon rigid-body name.

## Hardware Mode Commands

Use real Crazyflies, Crazyradio, and Vicon through `motion_capture_tracking`.
Open two terminals for each staged test.

### One Crazyflie

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source /home/prachit/gcbf_experiments_ws/install/local_setup.bash
ros2 launch crazyflie_vicon_bringup bringup_single.launch.py
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source /home/prachit/gcbf_experiments_ws/install/local_setup.bash
ros2 run crazyflie_vicon_bringup gcbf_experiments single_takeoff
```

Then test one-drone `cmdFullState` tracking:

```bash
ros2 run crazyflie_vicon_bringup gcbf_experiments single_setpoint
```

### Three Crazyflies

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source /home/prachit/gcbf_experiments_ws/install/local_setup.bash
ros2 launch crazyflie_vicon_bringup bringup_3.launch.py
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source /home/prachit/gcbf_experiments_ws/install/local_setup.bash
ros2 run crazyflie_vicon_bringup gcbf_experiments triple_takeoff
```

### Eight Crazyflies

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source /home/prachit/gcbf_experiments_ws/install/local_setup.bash
ros2 launch crazyflie_vicon_bringup bringup_8.launch.py
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source /home/prachit/gcbf_experiments_ws/install/local_setup.bash
ros2 run crazyflie_vicon_bringup gcbf_experiments eight_takeoff
```

Full position exchange:

```bash
ros2 run crazyflie_vicon_bringup gcbf_experiments gcbf8_position_exchange
```

## Simulation Mode Commands

Use simulation before hardware whenever changing the script or model. Open two
terminals for each staged test.

### One Crazyflie Simulation

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source /home/prachit/gcbf_experiments_ws/install/local_setup.bash
ros2 launch crazyflie_vicon_bringup bringup_single.launch.py backend:=sim mocap:=False rviz:=True gui:=True
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source /home/prachit/gcbf_experiments_ws/install/local_setup.bash
ros2 run crazyflie_vicon_bringup gcbf_experiments single_takeoff --ros-args -p use_sim_time:=True
```

One-drone simulated `cmdFullState` tracking:

```bash
ros2 run crazyflie_vicon_bringup gcbf_experiments single_setpoint --ros-args -p use_sim_time:=True
```

### Three Crazyflies Simulation

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source /home/prachit/gcbf_experiments_ws/install/local_setup.bash
ros2 launch crazyflie_vicon_bringup bringup_3.launch.py backend:=sim mocap:=False rviz:=True gui:=True
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source /home/prachit/gcbf_experiments_ws/install/local_setup.bash
ros2 run crazyflie_vicon_bringup gcbf_experiments triple_takeoff --ros-args -p use_sim_time:=True
```

### Eight Crazyflies Simulation

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source /home/prachit/gcbf_experiments_ws/install/local_setup.bash
ros2 launch crazyflie_vicon_bringup bringup_8.launch.py backend:=sim mocap:=False rviz:=True gui:=True
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source /home/prachit/gcbf_experiments_ws/install/local_setup.bash
ros2 run crazyflie_vicon_bringup gcbf_experiments eight_takeoff --ros-args -p use_sim_time:=True
```

Full simulated position exchange:

```bash
ros2 run crazyflie_vicon_bringup gcbf_experiments gcbf8_position_exchange --ros-args -p use_sim_time:=True
```

## Verify Vicon

```bash
ros2 topic echo /poses
ros2 topic echo /Minhyuk/pose
ros2 topic echo /cf5/pose
ros2 topic echo /cf7/pose
```

For placeholder drones, replace topic names with the real Vicon/Crazyflie names
after updating `crazyflies_8.yaml`.

For a more readable pose stream:

```bash
ros2 run crazyflie_vicon_bringup pose_debug --ros-args -p topic:=/Minhyuk/pose
```

To visualize another Crazyflie pose topic in the terminal, replace the `topic`
parameter:

```bash
ros2 run crazyflie_vicon_bringup pose_debug --ros-args -p topic:=/cf5/pose
```

## Model Replacement

The placeholder model path is:

```text
src/crazyflie_vicon_bringup/models/gcbfplus_policy.pt
```

Replace it with a trained TorchScript model and rebuild/source the workspace.
Expected model interface:

```text
input:  [N, 9] = px, py, pz, vx, vy, vz, gx, gy, gz
output: [N, 3] = ax, ay, az
```
