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
  --output gcbf_crazyswarm_double_integrator.npz
```

```bash
ls -lh /home/prachit/gcbf_experiments_ws/gcbf_crazyswarm_double_integrator.npz
```
