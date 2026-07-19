# Vicon 8-Crazyflie Position Exchange Setup

This package is the local overlay for the GCBF+ Crazyflie experiments. It uses
Crazyswarm2 with ROS 2 and Vicon through `motion_capture_tracking`.

## What This Uses

- ROS 2 Humble
- Crazyswarm2
- `motion_capture_tracking`
- Vicon host `10.192.46.131`
- `cmdFullState` streaming through `crazyflie_py`

This path does not use `vicon_bridge`, Jackal, Clearpath, rendezvous scripts, or
the old safety heartbeat nodes. The earlier issue was that Crazyswarm2 does not
consume `/vicon/...` topics directly. It consumes `poses` from
`motion_capture_tracking`, and the pose names must match the robot names in the
Crazyflie YAML file.

## Vicon Config

`config/motion_capture.yaml` should stay:

```yaml
/motion_capture_tracking:
  ros__parameters:
    type: "vicon"
    hostname: "10.192.46.131"

    topics:
      frame_id: "world"
      poses:
        qos:
          mode: "sensor"
          deadline: 100.0
      tf:
        child_frame_id: "{}"
```

## Build

```bash
source /opt/ros/humble/setup.bash
cd /home/james/Research/GCBF/GCBF_EXPERIMENTS_WS
colcon build --symlink-install
source install/local_setup.bash
```

## Configure Crazyflies

Scan powered Crazyflies:

```bash
source /opt/ros/humble/setup.bash
source /home/james/Research/GCBF/GCBF_EXPERIMENTS_WS/install/local_setup.bash
ros2 run crazyflie scan
```

Update these files as needed:

- `config/crazyflies_single.yaml`
- `config/crazyflies_3.yaml`
- `config/crazyflies_8.yaml`
- `config/crazyflies_8_debug_single.yaml`

Every robot key must exactly match the Vicon rigid-body name. Replace the TODO
placeholder names and URIs in `crazyflies_8.yaml` before real 8-drone tests.
For debug mode, `crazyflies_8_debug_single.yaml` keeps only `cf2` enabled by
default while preserving the full 8-drone Vicon layout.

## Hardware Mode Commands

Use these commands with real Crazyflies, Crazyradio, and Vicon through
`motion_capture_tracking`. Open two terminals for each staged test.

### One Crazyflie

Terminal 1:


```bash
source /opt/ros/humble/setup.bash
source /home/james/Research/GCBF/GCBF_EXPERIMENTS_WS/install/local_setup.bash
ros2 launch crazyflie_vicon_bringup bringup_single.launch.py
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source /home/james/Research/GCBF/GCBF_EXPERIMENTS_WS/install/local_setup.bash
ros2 run crazyflie_vicon_bringup gcbf_experiments single_takeoff
```

After that succeeds, test one-drone `cmdFullState` tracking:

```bash
ros2 run crazyflie_vicon_bringup gcbf_experiments single_setpoint
```

### Three Crazyflies

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source /home/james/Research/GCBF/GCBF_EXPERIMENTS_WS/install/local_setup.bash
ros2 launch crazyflie_vicon_bringup bringup_3.launch.py
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source /home/james/Research/GCBF/GCBF_EXPERIMENTS_WS/install/local_setup.bash
ros2 run crazyflie_vicon_bringup gcbf_experiments triple_takeoff
```

### Eight Crazyflies

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source /home/james/Research/GCBF/GCBF_EXPERIMENTS_WS/install/local_setup.bash
ros2 launch crazyflie_vicon_bringup bringup_8.launch.py
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source /home/james/Research/GCBF/GCBF_EXPERIMENTS_WS/install/local_setup.bash
ros2 run crazyflie_vicon_bringup gcbf_experiments eight_takeoff
```

Then run the full position exchange:

```bash
ros2 run crazyflie_vicon_bringup gcbf_experiments gcbf8_position_exchange
```

Wait for:

```text
All Crazyflies are fully connected!
```

### Eight-Crazyflie Debug Mode

Use this mode to test one active Crazyflie in the 8-drone Vicon layout before
running all eight. The default active drone is `cf2`.

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source /home/james/Research/GCBF/GCBF_EXPERIMENTS_WS/install/local_setup.bash
ros2 launch crazyflie_vicon_bringup bringup_8_debug_single.launch.py
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source /home/james/Research/GCBF/GCBF_EXPERIMENTS_WS/install/local_setup.bash
ros2 run crazyflie_vicon_bringup gcbf_experiments eight_debug_single_takeoff
```

This command waits for Vicon poses named `cf1` through `cf8`, checks that the
active estimator matches Vicon, then arms only `cf2` and streams a vertical
takeoff/land command. It triggers emergency stop if the debug altitude limit is
exceeded.

After the single-drone debug passes with all eight drones enabled in
`bringup_8.launch.py`, use:

```bash
ros2 run crazyflie_vicon_bringup gcbf_experiments eight_debug_takeoff
ros2 run crazyflie_vicon_bringup gcbf_experiments eight_debug_sequence
```

`eight_debug_takeoff` runs the debug takeoff/land on `cf2` with all eight drones
enabled. `eight_debug_sequence` runs the same debug takeoff/land one Crazyflie
at a time across all eight enabled drones.

## Simulation Mode Commands

Use simulation before hardware whenever changing the script or model. Open two
terminals for each staged test.

### One Crazyflie Simulation

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source /home/james/Research/GCBF/GCBF_EXPERIMENTS_WS/install/local_setup.bash
ros2 launch crazyflie_vicon_bringup bringup_single.launch.py backend:=sim mocap:=False rviz:=True gui:=True
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source /home/james/Research/GCBF/GCBF_EXPERIMENTS_WS/install/local_setup.bash
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
source /home/james/Research/GCBF/GCBF_EXPERIMENTS_WS/install/local_setup.bash
ros2 launch crazyflie_vicon_bringup bringup_3.launch.py backend:=sim mocap:=False rviz:=True gui:=True
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source /home/james/Research/GCBF/GCBF_EXPERIMENTS_WS/install/local_setup.bash
ros2 run crazyflie_vicon_bringup gcbf_experiments triple_takeoff --ros-args -p use_sim_time:=True
```

### Eight Crazyflies Simulation

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source /home/james/Research/GCBF/GCBF_EXPERIMENTS_WS/install/local_setup.bash
ros2 launch crazyflie_vicon_bringup bringup_8.launch.py backend:=sim mocap:=False rviz:=True gui:=True
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source /home/james/Research/GCBF/GCBF_EXPERIMENTS_WS/install/local_setup.bash
ros2 run crazyflie_vicon_bringup gcbf_experiments eight_takeoff --ros-args -p use_sim_time:=True
```

Full simulated position exchange:

```bash
ros2 run crazyflie_vicon_bringup gcbf_experiments gcbf8_position_exchange --ros-args -p use_sim_time:=True
```

## Verify Vicon

```bash
ros2 topic echo /poses
ros2 topic echo /cf8/pose
ros2 topic echo /cf5/pose
ros2 topic echo /cf7/pose
```

For the placeholder drones, replace the topic names with the real Vicon/Crazyflie
names after updating `crazyflies_8.yaml`.

For a more readable pose stream:

```bash
ros2 run crazyflie_vicon_bringup pose_debug --ros-args -p topic:=/cf8/pose
```

## Model Replacement

The placeholder model path is:

```text
models/gcbfplus_policy.pt
```

Replace it with a trained TorchScript model and rebuild/source the workspace.
Expected model interface:

```text
input:  [N, 9] = px, py, pz, vx, vy, vz, gx, gy, gz
output: [N, 3] = ax, ay, az
```

If PyTorch is unavailable, the model cannot be loaded, or this dummy file is
still present, the code falls back to a conservative nominal PD controller.

## Future Work

- Replace the fallback controller with the real GCBF+ policy.
- Tune `kp`, `kd`, `max_acc`, `safe_radius`, and circle radius from the training
  configuration.
- Validate 8-drone position exchange in simulation or staged/tethered hardware
  before free flight.
