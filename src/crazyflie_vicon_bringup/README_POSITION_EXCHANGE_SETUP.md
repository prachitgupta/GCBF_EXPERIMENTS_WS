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

- `config/crazyflies_single.yaml`
- `config/crazyflies_3.yaml`
- `config/crazyflies_8.yaml`

Every robot key must exactly match the Vicon rigid-body name. Replace the TODO
placeholder names and URIs in `crazyflies_8.yaml` before real 8-drone tests.

## Launch

Single drone:

```bash
ros2 launch crazyflie_vicon_bringup bringup_single.launch.py
```

Three drones:

```bash
ros2 launch crazyflie_vicon_bringup bringup_3.launch.py
```

Eight drones:

```bash
ros2 launch crazyflie_vicon_bringup bringup_8.launch.py
```

The launch files default to real hardware:

```bash
backend:=cflib mocap:=True
```

For Crazyswarm2 simulation, switch the backend and disable mocap:

```bash
ros2 launch crazyflie_vicon_bringup bringup_single.launch.py backend:=sim mocap:=False
ros2 launch crazyflie_vicon_bringup bringup_3.launch.py backend:=sim mocap:=False
ros2 launch crazyflie_vicon_bringup bringup_8.launch.py backend:=sim mocap:=False
```

Wait for:

```text
All Crazyflies are fully connected!
```

## Verify Vicon

```bash
ros2 topic echo /poses
ros2 topic echo /Minhyuk/pose
ros2 topic echo /cf5/pose
ros2 topic echo /cf7/pose
```

For the placeholder drones, replace the topic names with the real Vicon/Crazyflie
names after updating `crazyflies_8.yaml`.

## Run Experiments

Single takeoff:

```bash
ros2 run crazyflie_vicon_bringup gcbf_experiments single_takeoff
```

Three-drone takeoff:

```bash
ros2 run crazyflie_vicon_bringup gcbf_experiments triple_takeoff
```

Eight-drone takeoff:

```bash
ros2 run crazyflie_vicon_bringup gcbf_experiments eight_takeoff
```

One-drone `cmdFullState` setpoint tracking:

```bash
ros2 run crazyflie_vicon_bringup gcbf_experiments single_setpoint
```

Eight-drone GCBF+ position exchange placeholder:

```bash
ros2 run crazyflie_vicon_bringup gcbf_experiments gcbf8_position_exchange
```

For simulation runs, pass ROS sim time to the experiment command:

```bash
ros2 run crazyflie_vicon_bringup gcbf_experiments single_takeoff --ros-args -p use_sim_time:=True
ros2 run crazyflie_vicon_bringup gcbf_experiments single_setpoint --ros-args -p use_sim_time:=True
ros2 run crazyflie_vicon_bringup gcbf_experiments gcbf8_position_exchange --ros-args -p use_sim_time:=True
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
