# Crazyflie Vicon Bringup

This package keeps local Crazyflie bringup config outside the upstream
`crazyswarm2` repository.

Current assumptions:

- Vicon host is `10.192.46.131`
- Crazyflie ROS names must match Vicon rigid-body names
- Onboard controller is forced to PID (`stabilizer.controller: 1`)
- Estimator is Kalman (`stabilizer.estimator: 2`)

Authoritative setup guide:

- `README_POSITION_EXCHANGE_SETUP.md`

Files to update before flight:

- `config/crazyflies_single.yaml`
- `config/crazyflies_3.yaml`
- `config/crazyflies_8.yaml`
  - `config/crazyflies_8_debug_single.yaml` for single-drone debug in the 8-drone Vicon layout
  - Set the correct Crazyflie `uri` values
  - Keep every robot name equal to its Vicon rigid-body name
- `config/motion_capture.yaml`
  - Update `hostname` if the Vicon server address changes

Build note:

- You do not need to rebuild for README-only changes.
- Rebuild before hardware runs if Python, launch, config, package metadata, or
  `crazyswarm2` files changed.

```bash
cd /home/james/Research/GCBF/GCBF_EXPERIMENTS_WS
conda deactivate
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
```

Useful commands after build:

```bash
source /opt/ros/humble/setup.bash
source /home/james/Research/GCBF/GCBF_EXPERIMENTS_WS/install/local_setup.bash

ros2 launch crazyflie_vicon_bringup bringup_single.launch.py
ros2 launch crazyflie_vicon_bringup bringup_3.launch.py
ros2 launch crazyflie_vicon_bringup bringup_4.launch.py
ros2 launch crazyflie_vicon_bringup bringup_8.launch.py
ros2 launch crazyflie_vicon_bringup bringup_8_debug_single.launch.py

ros2 run crazyflie_vicon_bringup gcbf_experiments single_takeoff
ros2 run crazyflie_vicon_bringup gcbf_experiments single_setpoint
ros2 run crazyflie_vicon_bringup gcbf_experiments triple_takeoff
ros2 run crazyflie_vicon_bringup gcbf_experiments eight_takeoff
ros2 run crazyflie_vicon_bringup gcbf_experiments eight_debug_single_takeoff
ros2 run crazyflie_vicon_bringup gcbf_experiments eight_debug_takeoff
ros2 run crazyflie_vicon_bringup gcbf_experiments eight_debug_sequence
ros2 run crazyflie_vicon_bringup gcbf_experiments gcbf8_position_exchange
ros2 run crazyflie_vicon_bringup four_drone_setpoint
ros2 run crazyflie_vicon_bringup eight_drone_hover
```

Debug mode:

- `bringup_8_debug_single.launch.py` uses `config/crazyflies_8_debug_single.yaml`.
- The default active hardware drone is `cf2`; the other seven entries remain disabled but their Vicon rigid-body names must still publish poses.
- `eight_debug_single_takeoff` arms only the active `cf2`, checks estimator/Vicon agreement, streams a vertical debug takeoff/land, and stops if the altitude limit is exceeded.
- Use `eight_debug_takeoff` only after all eight drones are enabled in the normal 8-drone bringup.
- Use `eight_debug_sequence` to run the same debug takeoff/land one Crazyflie at a time across all eight enabled drones.
- `four_drone_setpoint` expects `cf1`, `cf2`, `cf3`, and `cf4` active from `bringup_4.launch.py`.
- `eight_drone_hover` expects all eight drones active from `bringup_8.launch.py`.
