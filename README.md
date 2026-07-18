# GCBF+ Crazyswarm2 Commands

```bash
cd /home/prachit/gcbf_experiments_ws
/usr/bin/python3 -m pip install --user -r src/gcbfplus/requirements.txt
/usr/bin/python3 -m pip install --user -e src/gcbfplus
```

```bash
cd /home/prachit/gcbf_experiments_ws
colcon build --symlink-install
source install/local_setup.bash
```

```bash
cd /home/prachit/gcbf_experiments_ws
source install/local_setup.bash
ros2 launch crazyflie_vicon_bringup bringup_8.launch.py backend:=sim mocap:=False rviz:=True gui:=True
```

```bash
cd /home/prachit/gcbf_experiments_ws
source install/local_setup.bash
ros2 launch gcbfplus gcbf_crazyswarm_nodes.launch.py \
  mode:=sim \
  rate_hz:=50.0 \
  lookahead_dt:=0.05 \
  hover_height:=0.5 \
  hover_epsilon:=0.06 \
  takeoff_timeout:=60.0 \
  goal_tolerance:=0.12 \
  max_runtime:=30.0 \
  area_size:=4.0 \
  max_action_age:=0.02 \
  print_latency:=false \
  save_error_plots:=true \
  save_animation:=true
```

The offboard node subscribes directly to simulator `/tf`, runs the policy from the newest complete eight-agent state at up to 50 Hz, and streams low-level `/cf*/cmd_full_state` commands for takeoff, policy flight, hold, and landing. It does not use the high-level takeoff or land services.

For Vicon hardware, first start the eight-Crazyflie real backend, then use the same robot configuration with `mode:=real`:

```bash
cd /home/prachit/gcbf_experiments_ws
source install/local_setup.bash
ros2 launch crazyflie_vicon_bringup bringup_8.launch.py backend:=cflib mocap:=True rviz:=True gui:=True
```

```bash
cd /home/prachit/gcbf_experiments_ws
source install/local_setup.bash
ros2 launch gcbfplus gcbf_crazyswarm_nodes.launch.py \
  mode:=real \
  rate_hz:=50.0 \
  lookahead_dt:=0.05 \
  hover_height:=0.5 \
  hover_epsilon:=0.06 \
  takeoff_timeout:=90.0 \
  goal_tolerance:=0.12 \
  max_runtime:=30.0 \
  area_size:=4.0 \
  max_action_age:=0.02 \
  print_latency:=false \
  save_error_plots:=false \
  save_animation:=false
```

`mode:=real` consumes coherent Vicon `/poses` batches, streams the same `/cf*/cmd_full_state` topics, arms before low-level takeoff, and disarms only after low-level landing is confirmed. Set `print_latency:=true` to print one-second timing summaries for state age, graph construction, policy and CBF inference, safety/lookahead work, command publication, and deadline misses.

```bash
cd /home/prachit/gcbf_experiments_ws
source install/local_setup.bash
ros2 topic hz /gcbf/state
ros2 topic hz /gcbf/action
ros2 topic hz /gcbf/cbf
ros2 topic hz /cf1/cmd_full_state
```

```bash
cd /home/prachit/gcbf_experiments_ws
source install/local_setup.bash
ros2 topic echo --once /gcbf/state
ros2 topic echo --once /gcbf/action
ros2 topic echo --once /gcbf/cbf
ros2 topic echo --once /cf1/cmd_full_state
```

```bash
ls -lh /home/prachit/gcbf_experiments_ws/src/gcbfplus/media/results/gcbf_crazyswarm_double_integrator.npz
ls -lh /home/prachit/gcbf_experiments_ws/src/gcbfplus/media/results/gcbf_crazyswarm_double_integrator.gif
ls -lh /home/prachit/gcbf_experiments_ws/src/gcbfplus/media/results/pose_errors
```

```bash
cd /home/prachit/gcbf_experiments_ws
/usr/bin/python3 - <<'PY'
import numpy as np

data = np.load("src/gcbfplus/media/results/gcbf_crazyswarm_double_integrator.npz")

pos = data["positions"]
goals = data["goals"]
actions = data["actions"]
min_dist = data["min_distance"]
cbf = data["cbf"]

final_xy = pos[-1, :, :2]
goal_err = np.linalg.norm(goals - final_xy, axis=1)

print("timesteps:", pos.shape[0])
print("min pairwise distance:", float(min_dist.min()))
print("final max goal error:", float(goal_err.max()))
print("final mean goal error:", float(goal_err.mean()))
print("min CBF value:", float(cbf.min()))
print("max action norm:", float(np.linalg.norm(actions, axis=-1).max()))
PY
```
