#!/usr/bin/env python3

import argparse
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from crazyflie_py import Crazyswarm
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.utilities import remove_ros_args
from tf2_msgs.msg import TFMessage


HEIGHT = 0.5
TAKEOFF_DURATION = 3.0
LAND_HEIGHT = 0.03
LAND_DURATION = 3.0
HOVER_TIME = 3.0
RATE_HZ = 50.0
LOOKAHEAD_DT = 0.05
CIRCLE_RADIUS = 1.0

# Standard placeholder values. TODO: replace/tune from the trained GCBF+
# model config once the real policy artifact is available.
KP = 1.0
KD = 1.2
MAX_ACC = 0.8
SAFE_RADIUS = 0.25
GOAL_TOLERANCE = 0.12
MAX_TRACKING_TIME = 30.0
POSE_TIMEOUT = 5.0
POSE_STALE_SECONDS = 1.0


class GcbfPolicy:
    """Small TorchScript adapter with nominal fallback until the NN is ready."""

    def __init__(self, model_path, logger):
        self.model = None
        self.logger = logger
        self.model_path = Path(model_path)
        self._load_model()

    def _load_model(self):
        if not self.model_path.exists():
            self.logger.warn(
                f'GCBF+ model path does not exist: {self.model_path}. '
                'Using nominal fallback controller.'
            )
            return

        try:
            import torch
        except ImportError:
            self.logger.warn('PyTorch is not available. Using nominal fallback controller.')
            return

        try:
            self.model = torch.jit.load(str(self.model_path), map_location='cpu')
            self.model.eval()
            self.logger.info(f'Loaded GCBF+ TorchScript model: {self.model_path}')
        except Exception as exc:
            self.logger.warn(
                f'Could not load GCBF+ model at {self.model_path}: {exc}. '
                'Using nominal fallback controller.'
            )
            self.model = None

    def accelerations(self, positions, velocities, goals):
        if self.model is None:
            return nominal_accelerations(positions, velocities, goals)

        try:
            import torch

            features = np.hstack((positions, velocities, goals)).astype(np.float32)
            with torch.no_grad():
                output = self.model(torch.from_numpy(features)).cpu().numpy()
            if output.shape != positions.shape:
                self.logger.warn(
                    f'GCBF+ model returned shape {output.shape}, expected '
                    f'{positions.shape}. Using nominal fallback controller.'
                )
                return nominal_accelerations(positions, velocities, goals)
            return clamp_rows(output, MAX_ACC)
        except Exception as exc:
            self.logger.warn(f'GCBF+ inference failed: {exc}. Using nominal fallback controller.')
            return nominal_accelerations(positions, velocities, goals)


def clamp_rows(values, max_norm):
    norms = np.linalg.norm(values, axis=1)
    scale = np.ones_like(norms)
    mask = norms > max_norm
    scale[mask] = max_norm / norms[mask]
    return values * scale[:, None]


def nominal_accelerations(positions, velocities, goals):
    accelerations = KP * (goals - positions) - KD * velocities
    return clamp_rows(accelerations, MAX_ACC)


def sorted_crazyflies(swarm):
    return sorted(swarm.allcfs.crazyflies, key=lambda cf: cf.prefix)


def attach_sim_pose_feedback(swarm, cfs):
    if not swarm.allcfs.get_parameter('use_sim_time').value:
        return

    crazyflies_by_frame = {cf.prefix.lstrip('/'): cf for cf in cfs}

    def tf_callback(msg):
        for transform in msg.transforms:
            cf = crazyflies_by_frame.get(transform.child_frame_id.lstrip('/'))
            if cf is None:
                continue

            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            cf.poseStamped_topic_callback(pose)

    swarm.allcfs.create_subscription(TFMessage, '/tf', tf_callback, 10)


def require_count(cfs, expected):
    if len(cfs) != expected:
        raise RuntimeError(f'Expected {expected} Crazyflies, found {len(cfs)}.')


def takeoff_and_land(cfs, time_helper):
    for cf in cfs:
        cf.takeoff(targetHeight=HEIGHT, duration=TAKEOFF_DURATION)
    time_helper.sleep(TAKEOFF_DURATION + HOVER_TIME)
    for cf in cfs:
        cf.land(targetHeight=LAND_HEIGHT, duration=LAND_DURATION)
    time_helper.sleep(LAND_DURATION + 1.0)


def wait_for_poses(cfs, time_helper, timeout=POSE_TIMEOUT):
    start = time_helper.time()
    while time_helper.time() - start < timeout:
        if all(cf.poseStamped for cf in cfs):
            return
        time_helper.sleepForRate(20.0)
    missing = [cf.prefix for cf in cfs if not cf.poseStamped]
    raise RuntimeError(f'Missing pose updates for: {", ".join(missing)}')


def current_positions(cfs):
    return np.array([cf.get_position() for cf in cfs], dtype=float)


def poses_are_stale(cfs, now):
    for cf in cfs:
        stamp = cf.poseStamped
        if not stamp:
            return True
        msg_time = float(stamp['timestamp_sec']) + float(stamp['timestamp_nsec']) * 1e-9
        if msg_time > 0.0 and now - msg_time > POSE_STALE_SECONDS:
            return True
    return False


def pairwise_too_close(positions, safe_radius):
    for i in range(len(positions)):
        for j in range(i + 1, len(positions)):
            if np.linalg.norm(positions[i] - positions[j]) < safe_radius:
                return True
    return False


def notify_setpoints_stop(cfs):
    for cf in cfs:
        cf.notifySetpointsStop()


def land_all(cfs, time_helper):
    for cf in cfs:
        cf.land(targetHeight=LAND_HEIGHT, duration=LAND_DURATION)
    time_helper.sleep(LAND_DURATION + 1.0)


def stream_setpoints(cfs, time_helper, goals, policy, max_time):
    wait_for_poses(cfs, time_helper)
    last_positions = current_positions(cfs)
    velocities = np.zeros_like(last_positions)
    start = time_helper.time()
    last_time = start

    while time_helper.time() - start < max_time:
        now = time_helper.time()
        positions = current_positions(cfs)
        sample_dt = max(now - last_time, 1.0 / RATE_HZ)
        velocities = (positions - last_positions) / sample_dt

        if poses_are_stale(cfs, now):
            raise RuntimeError('Pose data became stale.')
        if len(cfs) > 1 and pairwise_too_close(positions, SAFE_RADIUS):
            raise RuntimeError('Pairwise distance dropped below safe radius.')
        if np.all(np.linalg.norm(goals - positions, axis=1) < GOAL_TOLERANCE):
            break

        accelerations = policy.accelerations(positions, velocities, goals)
        velocity_refs = velocities + accelerations * LOOKAHEAD_DT
        position_refs = (
            positions
            + velocities * LOOKAHEAD_DT
            + 0.5 * accelerations * LOOKAHEAD_DT**2
        )

        for cf, pos_ref, vel_ref, acc in zip(cfs, position_refs, velocity_refs, accelerations):
            cf.cmdFullState(pos_ref, vel_ref, acc, 0.0, np.zeros(3))

        last_positions = positions
        last_time = now
        time_helper.sleepForRate(RATE_HZ)


def single_setpoint(swarm, cfs):
    require_count(cfs, 1)
    time_helper = swarm.timeHelper
    cf = cfs[0]
    cf.takeoff(targetHeight=HEIGHT, duration=TAKEOFF_DURATION)
    time_helper.sleep(TAKEOFF_DURATION + 1.0)

    model_path = default_model_path()
    policy = GcbfPolicy(model_path, swarm.allcfs.get_logger())

    try:
        wait_for_poses(cfs, time_helper)
        goal = current_positions(cfs)
        goal[0] = goal[0] + np.array([0.4, 0.0, 0.0])
        goal[0, 2] = HEIGHT
        stream_setpoints(cfs, time_helper, goal, policy, MAX_TRACKING_TIME)
    finally:
        notify_setpoints_stop(cfs)
        time_helper.sleep(0.2)
        land_all(cfs, time_helper)


def circle_positions(count):
    angles = np.linspace(0.0, 2.0 * np.pi, count, endpoint=False)
    positions = np.zeros((count, 3))
    positions[:, 0] = CIRCLE_RADIUS * np.cos(angles)
    positions[:, 1] = CIRCLE_RADIUS * np.sin(angles)
    positions[:, 2] = HEIGHT
    return positions


def ring_exchange_waypoints(starts):
    waypoints = []
    for step in range(1, 5):
        angle = step * np.pi / 4.0
        rotation = np.array(
            [
                [np.cos(angle), -np.sin(angle), 0.0],
                [np.sin(angle), np.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        waypoints.append(starts @ rotation.T)
    return waypoints


def execute_ring_exchange(cfs, time_helper, starts):
    for waypoint in ring_exchange_waypoints(starts):
        for cf, target in zip(cfs, waypoint):
            cf.goTo(target, 0.0, 2.0)
        time_helper.sleep(2.5)


def gcbf8_position_exchange(swarm, cfs):
    require_count(cfs, 8)
    time_helper = swarm.timeHelper
    starts = circle_positions(8)
    goals = np.roll(starts, shift=-4, axis=0)
    policy = GcbfPolicy(default_model_path(), swarm.allcfs.get_logger())

    try:
        for cf in cfs:
            cf.takeoff(targetHeight=HEIGHT, duration=TAKEOFF_DURATION)
        time_helper.sleep(TAKEOFF_DURATION + 1.0)

        for cf, start in zip(cfs, starts):
            cf.goTo(start, 0.0, 5.0)
        time_helper.sleep(6.0)

        if policy.model is None:
            swarm.allcfs.get_logger().warn(
                'Using collision-aware ring waypoints because no valid GCBF+ '
                'policy is available.'
            )
            execute_ring_exchange(cfs, time_helper, starts)
        else:
            stream_setpoints(cfs, time_helper, goals, policy, MAX_TRACKING_TIME)
    finally:
        notify_setpoints_stop(cfs)
        time_helper.sleep(0.2)
        land_all(cfs, time_helper)


def default_model_path():
    package_share = Path(get_package_share_directory('crazyflie_vicon_bringup'))
    return package_share / 'models' / 'gcbfplus_policy.pt'


def parse_args():
    parser = argparse.ArgumentParser(description='Crazyflie Vicon GCBF+ experiment helpers.')
    parser.add_argument(
        'mode',
        choices=[
            'single_takeoff',
            'triple_takeoff',
            'eight_takeoff',
            'single_setpoint',
            'gcbf8_position_exchange',
        ],
    )
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def main():
    args = parse_args()
    swarm = Crazyswarm()
    cfs = sorted_crazyflies(swarm)
    attach_sim_pose_feedback(swarm, cfs)
    time_helper = swarm.timeHelper

    try:
        if args.mode == 'single_takeoff':
            require_count(cfs, 1)
            takeoff_and_land(cfs, time_helper)
        elif args.mode == 'triple_takeoff':
            require_count(cfs, 3)
            takeoff_and_land(cfs, time_helper)
        elif args.mode == 'eight_takeoff':
            require_count(cfs, 8)
            takeoff_and_land(cfs, time_helper)
        elif args.mode == 'single_setpoint':
            single_setpoint(swarm, cfs)
        elif args.mode == 'gcbf8_position_exchange':
            gcbf8_position_exchange(swarm, cfs)
    except KeyboardInterrupt:
        notify_setpoints_stop(cfs)
        land_all(cfs, time_helper)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
