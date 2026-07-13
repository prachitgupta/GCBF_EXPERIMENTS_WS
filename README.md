# GCBF+ Crazyswarm2 Sim Commands

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
/usr/bin/python3 -m gcbfplus.crazyswarm_double_integrator \
  --output src/gcbfplus/media/results/gcbf_crazyswarm_double_integrator.npz \
  --save-error-plots \
  --error-plot-dir src/gcbfplus/media/results/pose_errors \
  --save-animation \
  --animation-file src/gcbfplus/media/results/gcbf_crazyswarm_double_integrator.gif
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
