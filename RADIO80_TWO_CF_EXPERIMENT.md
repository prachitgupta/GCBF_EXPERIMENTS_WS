# One Crazyradio / Two Crazyflie Experiment

This experiment tests independent unicast control of two Crazyflies through
one Crazyradio. Both links use radio index `0`, channel `80`, and 2 Mbit/s.
The Crazyflies remain independently addressable because their radio addresses
and ROS namespaces differ.

## Configurations

| Launch file | Robots | Purpose |
| --- | --- | --- |
| `radio80_single.launch.py` | `cf8` (`E7E7E7E708`) | Rehearse both sequential flight slots with one physical Crazyflie |
| `radio80_dual.launch.py` | `cf8`, `cf9` (`E7E7E7E708`, `E7E7E7E709`) | Test two physical Crazyflies through Crazyradio `0` |

Before testing, use CFclient to set each physical Crazyflie to channel `80`,
2 Mbit/s, and the address shown above. The Vicon rigid-body names must match
`cf8` and `cf9`.

## Build

```bash
cd /home/minhyuk/GCBF_EXPERIMENTS_WS
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select crazyflie_vicon_bringup
source install/local_setup.bash
```

## Stage 1: one-Crazyflie sequential rehearsal

Place `cf8` at the Vicon origin with clear space in both Y directions.

Terminal 1:

```bash
source /home/minhyuk/ros_domain_23.sh
source /opt/ros/humble/setup.bash
source /home/minhyuk/GCBF_EXPERIMENTS_WS/install/local_setup.bash
ros2 launch crazyflie_vicon_bringup radio80_single.launch.py
```

Wait for `All Crazyflies are fully connected!`, verify `/cf8/pose`, then run
the controller in Terminal 2:

```bash
source /home/minhyuk/ros_domain_23.sh
source /opt/ros/humble/setup.bash
source /home/minhyuk/GCBF_EXPERIMENTS_WS/install/local_setup.bash
ros2 run crazyflie_vicon_bringup radio80_sequential
```

The one Crazyflie flies slot 1, lands, then flies slot 2 and lands. This checks
the sequential command path, but it does not validate radio multiplexing.

## Stage 2: two-Crazyflie single-radio test

Place `cf8` at `[-0.5, 0.0, 0.0]` and `cf9` at `[0.5, 0.0, 0.0]`.

Start `radio80_dual.launch.py` in Terminal 1, using the same environment setup:

```bash
ros2 launch crazyflie_vicon_bringup radio80_dual.launch.py
```

Confirm both `/cf8/pose` and `/cf9/pose` update, then run:

```bash
ros2 run crazyflie_vicon_bringup radio80_sequential
```

Expected result: `cf8` flies and lands while `cf9` remains landed; then `cf9`
flies and lands while `cf8` remains landed. This demonstrates independent
unicast control over one Crazyradio before attempting simultaneous flight.
