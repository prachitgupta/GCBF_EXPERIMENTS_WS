# GCBF+ Crazyswarm2 Offboard Controller

The controller consumes the newest complete eight-agent state, runs fused JAX graph, policy, and CBF inference, and streams low-level `/cf*/cmd_full_state` commands at up to 50 Hz. It does not use the high-level takeoff or land services.

## Build

```bash
cd /path/to/gcbf_experiments_ws
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m pip install --user -r src/gcbfplus/requirements.txt
colcon build --packages-select gcbfplus --symlink-install
source install/local_setup.bash
```

The repository requirements use JAX and JAXlib `0.4.18`. Confirm the active backend before running:

```bash
/usr/bin/python3 -c 'import jax; print(jax.__version__); print(jax.default_backend()); print(jax.devices())'
```

## Reproduce the CPU simulation benchmark

Start the headless simulator in terminal 1:

```bash
cd /path/to/gcbf_experiments_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch crazyflie_vicon_bringup bringup_8.launch.py backend:=sim mocap:=False rviz:=False gui:=False ros_domain_id:=88
```

Start the controller and monitor in terminal 2. Every user-defined launch parameter is included explicitly:

```bash
cd /path/to/gcbf_experiments_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch gcbfplus gcbf_crazyswarm_nodes.launch.py \
  ros_domain_id:=88 \
  mode:=sim \
  rate_hz:=50.0 \
  lookahead_dt:=0.05 \
  hover_height:=0.5 \
  hover_epsilon:=0.06 \
  takeoff_timeout:=60.0 \
  goal_tolerance:=0.12 \
  max_runtime:=90.0 \
  area_size:=4.0 \
  car_radius:=0.05 \
  max_action_age:=0.02 \
  print_latency:=true \
  save_error_plots:=false \
  save_animation:=false
```

With the CPU JAX backend, the fused graph, policy, and CBF inference measured approximately 2.6 ms and the complete policy cycle approximately 4.2 ms, with zero 20 ms deadline misses. This run detected a collision and did not reach all goals; the latency optimization does not by itself resolve the remaining controller/dynamics safety issue.

## Run on Vicon hardware

The simulation launch automatically uses `crazyflies_8_sim.yaml`, preserving the validated 1.4 m simulation geometry.

Start the real backend in terminal 1. If the Crazyflie firmware Python bindings are not already installed, set `CRAZYFLIE_FIRMWARE_BINDINGS` to their build directory before launching:

```bash
cd /path/to/gcbf_experiments_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88
# export CRAZYFLIE_FIRMWARE_BINDINGS=/absolute/path/to/crazyflie-firmware/build
ros2 launch crazyflie_vicon_bringup bringup_8.launch.py \
  backend:=cflib \
  mocap:=True \
  rviz:=True \
  gui:=True \
  ros_domain_id:=88
```

After confirming all eight Vicon poses and the flight area are safe, start the same controller in terminal 2:

```bash
cd /path/to/gcbf_experiments_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch gcbfplus gcbf_crazyswarm_nodes.launch.py \
  ros_domain_id:=88 \
  mode:=real \
  rate_hz:=50.0 \
  lookahead_dt:=0.05 \
  hover_height:=0.5 \
  hover_epsilon:=0.06 \
  takeoff_timeout:=60.0 \
  goal_tolerance:=0.12 \
  max_runtime:=90.0 \
  area_size:=4.0 \
  car_radius:=0.05 \
  max_action_age:=0.02 \
  print_latency:=true \
  save_error_plots:=false \
  save_animation:=false
```

`mode:=sim` subscribes to `/tf`, uses simulation time, and selects `crazyflies_8_sim.yaml`. `mode:=real` subscribes to coherent `/poses` batches, selects James's hardware-tested `crazyflies_8.yaml`, arms before low-level takeoff, and disarms after low-level landing is confirmed. Both launches default to ROS domain 88 and also accept `ros_domain_id:=...` explicitly.

`car_radius` is the policy's radius for one agent; the centre-to-centre collision boundary is `2 * car_radius`. During policy execution, the actor prints the measured minimum pairwise distance and collision boundary once per second using the distance already calculated for its safety check.

## Observe the ROS streams

Run each command in a separate sourced terminal:

```bash
ros2 topic hz /gcbf/state
ros2 topic hz /gcbf/action
ros2 topic hz /gcbf/cbf
ros2 topic hz /cf1/cmd_full_state
```
