# Crazyflie Vicon Setup Notes

This document explains exactly what was done to set up the Crazyflie ROS 2 stack with Vicon on this machine, why those steps were needed, and how to repeat the process on another machine.

The target result of this setup is:

- use `crazyswarm2` as the Crazyflie ROS 2 stack
- use Vicon as the external positioning system
- use the onboard PID controller first, not Mellinger
- support the following first-flight sequence:
  - connect to the Crazyflie
  - take off
  - hover
  - land
  - teleoperate in hover
  - run the figure-8 demo

## 1. High-level conclusion

The most important point is this:

- your existing `vicon_bridge` setup was not enough by itself for `crazyswarm2`

Why:

- your Vicon bridge publishes topics like `/vicon/Minhyuk/Minhyuk` and `/vicon/Minhyuk/Minhyuk/pose`
- `crazyswarm2` does **not** directly consume those topics for flight
- instead, `crazyswarm2` expects the ROS package `motion_capture_tracking` to publish a `NamedPoseArray` stream on the `poses` topic
- the `crazyflie_server` node subscribes to that `poses` topic and matches incoming pose names against the robot names in `crazyflies.yaml`

So even if Vicon was already working in ROS through `vicon_bridge`, the Crazyflie stack still needed the separate `motion_capture_tracking` package because that is the interface `crazyswarm2` was built around.

## 2. Official references used

These are the main references I followed:

- Crazyswarm2 repository:
  https://github.com/IMRCLab/crazyswarm2
- Crazyswarm2 installation docs:
  https://imrclab.github.io/crazyswarm2/installation.html
- Crazyswarm2 usage docs:
  https://imrclab.github.io/crazyswarm2/usage.html
- Bitcraze USB permission documentation:
  https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/

The local clone of the Crazyflie stack is here:

- [`/home/minhyuk/crazyflie_ws/src/crazyswarm2`](/home/minhyuk/crazyflie_ws/src/crazyswarm2)

## 3. Workspaces that were created

Two separate ROS 2 workspaces were created on purpose.

### 3.1 Crazyflie base workspace

Path:

- [`/home/minhyuk/crazyflie_ws`](/home/minhyuk/crazyflie_ws)

Purpose:

- install and build the upstream Crazyflie stack
- keep the Vicon/Crazyflie bringup together
- keep custom config separate from upstream source

### 3.2 Future follower workspace

Path:

- [`/home/minhyuk/jackal_cf_follow_ws`](/home/minhyuk/jackal_cf_follow_ws)

Purpose:

- later hold the Jackal-following logic
- keep experimental follower code isolated from the base Crazyflie bringup
- make it easier to publish follower code to GitHub later

The initial follower package created there is:

- [`/home/minhyuk/jackal_cf_follow_ws/src/jackal_cf_follow`](/home/minhyuk/jackal_cf_follow_ws/src/jackal_cf_follow)

## 4. Packages and dependencies installed

### 4.1 Crazyswarm2 source stack

The official ROS 2 Crazyflie stack was cloned into:

- [`/home/minhyuk/crazyflie_ws/src/crazyswarm2`](/home/minhyuk/crazyflie_ws/src/crazyswarm2)

Submodules were initialized because the stack depends on nested repositories such as Crazyflie communication libraries.

### 4.2 Python packages

Installed for the current user:

- `rowan`
- `nicegui==1.4.2`
- `cflib`
- `transforms3d`

These are required by the Crazyflie ROS 2 stack and the Bitcraze Python communication layer.

### 4.3 System and ROS packages

Installed:

- `libboost-program-options-dev`
- `libusb-1.0-0-dev`
- `ros-humble-tf-transformations`
- `ros-humble-teleop-twist-keyboard`
- `ros-humble-motion-capture-tracking`
- `ros-humble-motion-capture-tracking-interfaces`

The heaviest install was `ros-humble-motion-capture-tracking`, because it pulled in a large dependency chain including PCL and related libraries.

## 5. Why `motion_capture_tracking` was required

This is the part that usually confuses people.

### 5.1 What you already had

You already had:

- a working Vicon system
- a ROS 2 Vicon bridge
- published topics like:
  - `/vicon/Minhyuk/Minhyuk`
  - `/vicon/Minhyuk/Minhyuk/pose`

That means Vicon data was getting into ROS 2.

### 5.2 Why that still was not enough

`crazyswarm2` is not written to consume arbitrary Vicon pose topics directly.

Internally, `crazyflie_server` subscribes to a motion-capture pose stream on the topic:

- `poses`

That stream is expected to be of type:

- `motion_capture_tracking_interfaces/msg/NamedPoseArray`

The incoming pose name must match the configured Crazyflie name in `crazyflies.yaml`.

So the needed chain is:

1. `motion_capture_tracking` connects to the Vicon server directly
2. it publishes pose messages in the format `crazyswarm2` expects
3. `crazyflie_server` receives those pose updates
4. `crazyflie_server` forwards external pose information over radio to the Crazyflie

### 5.3 Why I did not reuse `vicon_bridge` directly

I did not use your existing `vicon_bridge` as the main positioning input for `crazyswarm2` because:

- the topic format is different
- `crazyswarm2` already has a standard supported path through `motion_capture_tracking`
- using the supported path is easier to reproduce, easier to debug, and much closer to the official documentation

## 6. How the Vicon IP was found and configured

You said earlier that Vicon was already working with:

```bash
ros2 launch vicon_bridge all_segments.launch.py
```

So I inspected your Vicon bridge launch file here:

- [`/home/minhyuk/colcon_ws/src/ros2-vicon-bridge/launch/all_segments.launch.py`](/home/minhyuk/colcon_ws/src/ros2-vicon-bridge/launch/all_segments.launch.py)

That file had:

```python
vicon_computer_ip = "10.192.46.131"
port_number = "801"
```

From that, I reused the same Vicon host in the Crazyflie motion-capture configuration.

The IP was then configured here:

- [`/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/config/motion_capture.yaml`](/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/config/motion_capture.yaml)

The exact setting is:

```yaml
/motion_capture_tracking:
  ros__parameters:
    type: "vicon"
    hostname: "10.192.46.131"
```

That is the key location you were missing earlier.

If you repeat this on another machine:

1. find the IP or hostname of the Vicon tracking computer
2. set `type: "vicon"`
3. set `hostname: "<your_vicon_ip_or_hostname>"`
4. rebuild or relaunch the workspace

## 7. Custom local bringup package that was created

Instead of editing the upstream `crazyswarm2` package directly, I created a local overlay package:

- [`/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup`](/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup)

This package exists so that:

- your local configuration is not mixed into upstream code
- updates to `crazyswarm2` are easier later
- your custom launch/config files stay in one place

### 7.1 Files inside that package

Main files:

- [`/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/config/crazyflies.yaml`](/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/config/crazyflies.yaml)
- [`/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/config/motion_capture.yaml`](/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/config/motion_capture.yaml)
- [`/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/launch/bringup.launch.py`](/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/launch/bringup.launch.py)
- [`/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/launch/teleop_hover.launch.py`](/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/launch/teleop_hover.launch.py)

## 8. How the Crazyflie was configured

The main configuration file is:

- [`/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/config/crazyflies.yaml`](/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/config/crazyflies.yaml)

### 8.1 Robot name

The robot name was set to:

- `Minhyuk`

That was intentional.

Why:

- the motion-capture system publishes rigid body names
- `crazyflie_server` matches incoming mocap pose names against the robot names in `crazyflies.yaml`
- so the Crazyflie name and the Vicon rigid-body name must match

At the moment, the config uses:

```yaml
robots:
  Minhyuk:
```

If your Vicon rigid body is renamed later, this file must be updated to the exact same name.

### 8.2 URI placeholder

The current URI in the file is still a placeholder:

```yaml
uri: radio://0/80/2M/E7E7E7E7E7
```

That must be replaced with the real URI discovered from the radio scan when the Crazyflie is powered on.

### 8.3 Controller selection

You specifically remembered needing to change from Mellinger to PID. That is correct for this first stage.

The relevant setting is here:

```yaml
all:
  firmware_params:
    stabilizer:
      estimator: 2
      controller: 1
```

Meaning:

- `estimator: 2` means Kalman
- `controller: 1` means PID

This is in:

- [`/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/config/crazyflies.yaml`](/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/config/crazyflies.yaml)

This choice was made because PID is safer for initial mocap bringup and better aligned with your earlier working memory of the hardware.

## 9. Launch files that were created

### 9.1 Base bringup

Launch file:

- [`/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/launch/bringup.launch.py`](/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/launch/bringup.launch.py)

Purpose:

- launch `crazyflie` with:
  - `backend := cflib`
  - `mocap := true`
  - local `crazyflies.yaml`
  - local `motion_capture.yaml`

### 9.2 Teleop hover

Launch file:

- [`/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/launch/teleop_hover.launch.py`](/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/launch/teleop_hover.launch.py)

Purpose:

- launch the same Vicon-backed Crazyflie stack
- also launch `vel_mux.py`
- support keyboard teleop on a fixed hover height

The `robot_prefix` in that launch file is:

- `/Minhyuk`

## 10. Jackal-following prototype

After the base Crazyflie + Vicon bringup was working, a separate follower workspace was used to prototype the first Jackal-following behavior:

- [`/home/minhyuk/jackal_cf_follow_ws`](/home/minhyuk/jackal_cf_follow_ws)

Main package:

- [`/home/minhyuk/jackal_cf_follow_ws/src/jackal_cf_follow`](/home/minhyuk/jackal_cf_follow_ws/src/jackal_cf_follow)

Important files:

- [`/home/minhyuk/jackal_cf_follow_ws/src/jackal_cf_follow/jackal_cf_follow/circle_follower.py`](/home/minhyuk/jackal_cf_follow_ws/src/jackal_cf_follow/jackal_cf_follow/circle_follower.py)
- [`/home/minhyuk/jackal_cf_follow_ws/src/jackal_cf_follow/launch/circle_follow.launch.py`](/home/minhyuk/jackal_cf_follow_ws/src/jackal_cf_follow/launch/circle_follow.launch.py)

The current follower path:

- waits for both Vicon poses
- takes off the Crazyflie
- tries to move the Crazyflie directly above the Jackal
- only then starts the Jackal circle

The first follower version used absolute position setpoints.

That proved too brittle, so the follower was changed to publish world-frame Crazyflie velocity commands instead. The Jetson-side follower is therefore currently a soft proportional velocity controller layered on top of the Crazyflie onboard PID.

During debugging, the follower was also updated to include:

- softer gains
- speed limits
- Crazyflie Vicon jump rejection
- low-pass filtering of accepted Crazyflie Vicon pose samples

Current conclusion:

- the basic software bringup is present
- the main remaining limiter is the quality and stability of the Crazyflie Vicon pose during follower tests

See:

- [OPEN_PROBLEMS.md](/home/minhyuk/jackal_ros2_explanation_files/OPEN_PROBLEMS.md)

That must stay consistent with the Crazyflie name in `crazyflies.yaml`.

## 10. Build and verification steps that were completed

### 10.1 Build of Crazyflie workspace

Built successfully:

- [`/home/minhyuk/crazyflie_ws`](/home/minhyuk/crazyflie_ws)

Command used:

```bash
source /opt/ros/humble/setup.bash
cd /home/minhyuk/crazyflie_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 10.2 Build of future follower workspace

Built successfully:

- [`/home/minhyuk/jackal_cf_follow_ws`](/home/minhyuk/jackal_cf_follow_ws)

Command used:

```bash
source /opt/ros/humble/setup.bash
cd /home/minhyuk/jackal_cf_follow_ws
colcon build --symlink-install
```

### 10.3 Package visibility check

Verified that these packages are visible after sourcing the workspace:

- `crazyflie`
- `crazyflie_examples`
- `crazyflie_vicon_bringup`

## 11. Crazyradio permission fix

Initially, scanning for Crazyflies failed with:

- `Access denied (insufficient permissions)`

That meant the USB dongle was present, but the user did not yet have proper non-root access to it.

The Crazyradio was visible with:

```bash
lsusb
```

and showed:

- `1915:7777`

That is the Bitcraze Crazyradio PA USB identifier.

I then installed a udev rule so the current user can access it through the `plugdev` group.

The rule was written to:

- [`/etc/udev/rules.d/99-crazyradio.rules`](/etc/udev/rules.d/99-crazyradio.rules)

with:

```udev
SUBSYSTEM=="usb", ATTR{idVendor}=="1915", ATTR{idProduct}=="7777", MODE="0664", GROUP="plugdev"
```

After reloading udev, scanning no longer failed with permissions. It returned:

```text
[]
```

That is expected if no Crazyflie is powered on. It means:

- the radio dongle is accessible
- the remaining missing piece is simply a powered-on Crazyflie

## 12. How to repeat this on another machine

This is the shortest reproducible path.

### 12.1 Install ROS 2 Humble first

Make sure ROS 2 Humble is installed and working.

### 12.2 Create the workspaces

```bash
mkdir -p ~/crazyflie_ws/src
mkdir -p ~/jackal_cf_follow_ws/src
```

### 12.3 Clone Crazyswarm2

```bash
cd ~/crazyflie_ws/src
git clone https://github.com/IMRCLab/crazyswarm2.git
cd crazyswarm2
git submodule sync
git submodule update --init --recursive
```

### 12.4 Install dependencies

```bash
sudo apt-get install -y \
  libboost-program-options-dev \
  libusb-1.0-0-dev \
  ros-humble-tf-transformations \
  ros-humble-teleop-twist-keyboard \
  ros-humble-motion-capture-tracking \
  ros-humble-motion-capture-tracking-interfaces

python3 -m pip install --user rowan nicegui==1.4.2 cflib transforms3d
```

### 12.5 Create a local bringup package

Do not edit the upstream package directly if you can avoid it.

Create your own package for:

- `crazyflies.yaml`
- `motion_capture.yaml`
- custom launch files

This is exactly why `crazyflie_vicon_bringup` was created here.

### 12.6 Set the Vicon IP

In your local `motion_capture.yaml`, set:

```yaml
/motion_capture_tracking:
  ros__parameters:
    type: "vicon"
    hostname: "<your_vicon_ip>"
```

### 12.7 Set the Crazyflie controller to PID

In your local `crazyflies.yaml`, set:

```yaml
all:
  firmware_params:
    stabilizer:
      estimator: 2
      controller: 1
```

### 12.8 Keep the robot name equal to the mocap rigid-body name

Example:

```yaml
robots:
  Minhyuk:
    enabled: true
    uri: radio://0/80/2M/E7E7E7E7E7
    initial_position: [0.0, 0.0, 0.0]
    type: cf_vicon
```

If the Vicon rigid body is named `Minhyuk`, then the robot name must also be `Minhyuk`.

### 12.9 Build the workspace

```bash
source /opt/ros/humble/setup.bash
cd ~/crazyflie_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 12.10 Install the Crazyradio udev rule

Create:

```bash
sudo tee /etc/udev/rules.d/99-crazyradio.rules >/dev/null <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="1915", ATTR{idProduct}=="7777", MODE="0664", GROUP="plugdev"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

If needed, unplug and replug the Crazyradio dongle.

### 12.11 Power on the Crazyflie and scan

```bash
source /opt/ros/humble/setup.bash
source ~/crazyflie_ws/install/local_setup.bash
ros2 run crazyflie scan
```

Put the discovered URI into your local `crazyflies.yaml`.

### 12.12 Launch the Vicon-backed stack

```bash
source /opt/ros/humble/setup.bash
source ~/crazyflie_ws/install/local_setup.bash
ros2 launch crazyflie_vicon_bringup bringup.launch.py
```

For teleop hover:

```bash
ros2 launch crazyflie_vicon_bringup teleop_hover.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

For the basic scripted flight:

```bash
ros2 run crazyflie_examples hello_world
```

For figure-8:

```bash
ros2 run crazyflie_examples figure8
```

## 13. Current remaining live step on this machine

The Crazyradio is working and accessible, but the Crazyflie itself was not powered on during setup.

So the remaining live steps are:

1. power on the Crazyflie
2. run `ros2 run crazyflie scan`
3. update the URI in `crazyflies.yaml`
4. confirm the actual aircraft version and radio link
5. start Vicon-backed takeoff/hover/land tests

## 14. Most important files to remember

- Vicon bridge IP source:
  [`/home/minhyuk/colcon_ws/src/ros2-vicon-bridge/launch/all_segments.launch.py`](/home/minhyuk/colcon_ws/src/ros2-vicon-bridge/launch/all_segments.launch.py)
- Local Crazyflie Vicon config:
  [`/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/config/motion_capture.yaml`](/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/config/motion_capture.yaml)
- Local Crazyflie controller and robot config:
  [`/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/config/crazyflies.yaml`](/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/config/crazyflies.yaml)
- Local bringup launch:
  [`/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/launch/bringup.launch.py`](/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/launch/bringup.launch.py)
- Local teleop launch:
  [`/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/launch/teleop_hover.launch.py`](/home/minhyuk/crazyflie_ws/src/crazyflie_vicon_bringup/launch/teleop_hover.launch.py)
- Future follower workspace:
  [`/home/minhyuk/jackal_cf_follow_ws/src/jackal_cf_follow`](/home/minhyuk/jackal_cf_follow_ws/src/jackal_cf_follow)
