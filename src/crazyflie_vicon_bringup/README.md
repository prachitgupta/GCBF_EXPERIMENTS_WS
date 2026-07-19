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
  - Set the correct Crazyflie `uri` values
  - Keep every robot name equal to its Vicon rigid-body name
- `config/motion_capture.yaml`
  - Update `hostname` if the Vicon server address changes

Useful commands after build:

```bash
source /opt/ros/humble/setup.bash
source /path/to/gcbf_experiments_ws/install/local_setup.bash

ros2 launch crazyflie_vicon_bringup bringup_single.launch.py
ros2 launch crazyflie_vicon_bringup bringup_3.launch.py
ros2 launch crazyflie_vicon_bringup bringup_8.launch.py

ros2 run crazyflie_vicon_bringup pose_debug --ros-args -p topic:=/Minhyuk/pose
ros2 run crazyflie_vicon_bringup pose_debug --ros-args -p topic:=/cf5/pose
```
