# Multi-Crazyflie Demo Commands

Use this file when starting the three-Crazyflie tests.

Open separate terminals and paste one block into each terminal. These commands intentionally do not use background processes so you can see logs and stop each part cleanly.

## Sequential Takeoff / Land Test

Behavior:

- Requires the safety heartbeat before starting.
- Takes off `cf8`, then `cf5`, then `cf7`.
- Lands all three together.
- If the heartbeat terminal is stopped, all Crazyflies should land in place.

### Terminal 1: Crazyflie Bringup

```bash
source /home/minhyuk/ros_domain_23.sh
source /opt/ros/humble/setup.bash
source /home/minhyuk/crazyflie_ws/install/local_setup.bash
ros2 launch crazyflie_vicon_bringup bringup.launch.py
```

Wait for:

```text
All Crazyflies are fully connected!
```

### Terminal 2: Safety Heartbeat

```bash
source /home/minhyuk/ros_domain_23.sh
source /opt/ros/humble/setup.bash
source /home/minhyuk/jackal_cf_follow_ws/install/local_setup.bash
ros2 run jackal_cf_follow safety_guard
```

### Terminal 3: Sequential Takeoff / Land

```bash
source /home/minhyuk/ros_domain_23.sh
source /opt/ros/humble/setup.bash
source /home/minhyuk/crazyflie_ws/install/local_setup.bash
source /home/minhyuk/jackal_cf_follow_ws/install/local_setup.bash
ros2 launch jackal_cf_follow sequential_takeoff_land.launch.py
```

## Moon Demo

Behavior:

- Requires Vicon, Crazyflie bringup, and safety heartbeat.
- Takes off the Crazyflies.
- Moves them into formation above/around the Jackal.
- Drives the Jackal in one circle.
- Crazyflies orbit/follow the Jackal.
- Stops the Jackal.
- Lands the Crazyflies.
- If heartbeat is lost or Crazyflies get too close, the Jackal stops and the Crazyflies land in place.

Current tuned parameters:

```text
Jackal circle radius: 0.6 m
Jackal forward velocity: 0.2 m/s
Crazyflie moon radius around Jackal: 0.4 m
Minimum CF-CF separation before abort: 0.4 m
```

### Terminal 1: Vicon Bridge

```bash
source /home/minhyuk/ros_domain_23.sh
source /opt/ros/humble/setup.bash
source /home/minhyuk/colcon_ws/install/local_setup.bash
ros2 launch vicon_bridge all_segments.launch.py
```

### Terminal 2: Crazyflie Bringup

```bash
source /home/minhyuk/ros_domain_23.sh
source /opt/ros/humble/setup.bash
source /home/minhyuk/crazyflie_ws/install/local_setup.bash
ros2 launch crazyflie_vicon_bringup bringup.launch.py
```

Wait for:

```text
All Crazyflies are fully connected!
```

### Terminal 3: Safety Heartbeat

```bash
source /home/minhyuk/ros_domain_23.sh
source /opt/ros/humble/setup.bash
source /home/minhyuk/jackal_cf_follow_ws/install/local_setup.bash
ros2 run jackal_cf_follow safety_guard
```

### Terminal 4: Optional Pose Check

```bash
source /home/minhyuk/ros_domain_23.sh
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E '/(cf8|cf5|cf7|Jackal|poses)'
```

Optional individual checks:

```bash
ros2 topic echo /cf8/pose
```

```bash
ros2 topic echo /cf5/pose
```

```bash
ros2 topic echo /cf7/pose
```

```bash
ros2 topic echo /vicon/Jackal/Jackal/pose
```

### Terminal 5: Moon Follower

```bash
source /home/minhyuk/ros_domain_23.sh
source /opt/ros/humble/setup.bash
source /etc/clearpath/setup.bash
source /home/minhyuk/crazyflie_ws/install/local_setup.bash
source /home/minhyuk/jackal_cf_follow_ws/install/local_setup.bash
ros2 launch jackal_cf_follow moon_follow.launch.py
```

## Emergency Actions

Stop the safety heartbeat terminal:

```bash
Ctrl-C
```

Expected behavior:

- Jackal receives zero `cmd_vel`.
- Crazyflies receive land commands immediately and land in place.

If the moon follower terminal itself is stopped with `Ctrl-C`, it should also stop the Jackal and send land commands before exiting.
