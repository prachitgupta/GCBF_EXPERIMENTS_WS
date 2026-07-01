# GCBF+ Crazyflie Position Exchange Experiments

This workspace is set up to reproduce the 8-Crazyflie position exchange
experiment with a trained GCBF+ policy using Crazyswarm2.

The active experiment package is:

```text
src/crazyflie_vicon_bringup
```

The detailed setup and runbook is:

```text
README_POSITION_EXCHANGE_SETUP.md
```

Package-local copy:

```text
src/crazyflie_vicon_bringup/README_POSITION_EXCHANGE_SETUP.md
```

## Build

```bash
source /opt/ros/humble/setup.bash
cd /home/prachit/gcbf_experiments_ws
colcon build --symlink-install
source install/local_setup.bash
```

## Hardware Quick Start

Single Crazyflie:

```bash
ros2 launch crazyflie_vicon_bringup bringup_single.launch.py
ros2 run crazyflie_vicon_bringup gcbf_experiments single_takeoff
```

Three Crazyflies:

```bash
ros2 launch crazyflie_vicon_bringup bringup_3.launch.py
ros2 run crazyflie_vicon_bringup gcbf_experiments triple_takeoff
```

Eight Crazyflies:

```bash
ros2 launch crazyflie_vicon_bringup bringup_8.launch.py
ros2 run crazyflie_vicon_bringup gcbf_experiments eight_takeoff
ros2 run crazyflie_vicon_bringup gcbf_experiments gcbf8_position_exchange
```

## Simulation Quick Start

Single Crazyflie:

```bash
ros2 launch crazyflie_vicon_bringup bringup_single.launch.py backend:=sim mocap:=False rviz:=True gui:=True
ros2 run crazyflie_vicon_bringup gcbf_experiments single_takeoff --ros-args -p use_sim_time:=True
```

Three Crazyflies:

```bash
ros2 launch crazyflie_vicon_bringup bringup_3.launch.py backend:=sim mocap:=False rviz:=True gui:=True
ros2 run crazyflie_vicon_bringup gcbf_experiments triple_takeoff --ros-args -p use_sim_time:=True
```

Eight Crazyflies:

```bash
ros2 launch crazyflie_vicon_bringup bringup_8.launch.py backend:=sim mocap:=False rviz:=True gui:=True
ros2 run crazyflie_vicon_bringup gcbf_experiments eight_takeoff --ros-args -p use_sim_time:=True
ros2 run crazyflie_vicon_bringup gcbf_experiments gcbf8_position_exchange --ros-args -p use_sim_time:=True
```

## Notes

- The hardware path uses Crazyswarm2 with Vicon through
  `motion_capture_tracking`; it does not use `vicon_bridge`, Jackal,
  Clearpath, or rendezvous scripts.
- Robot names in the Crazyflie YAML files must exactly match the Vicon
  rigid-body names.
- Replace placeholder URIs in
  `src/crazyflie_vicon_bringup/config/crazyflies_8.yaml` before hardware
  8-drone tests.
- Replace `src/crazyflie_vicon_bringup/models/gcbfplus_policy.pt` with the
  trained GCBF+ TorchScript model before using the learned controller.
