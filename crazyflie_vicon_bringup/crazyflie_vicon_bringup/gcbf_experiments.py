#!/usr/bin/env python3

import argparse
import os
import sys
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = '88'

from ament_index_python.packages import get_package_share_directory
from crazyflie_py import Crazyswarm
from geometry_msgs.msg import PoseStamped
from motion_capture_tracking_interfaces.msg import NamedPoseArray
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from tf2_msgs.msg import TFMessage
import yaml


HEIGHT = 0.5
TAKEOFF_DURATION = 3.0
LAND_HEIGHT = 0.10
LAND_DURATION = 3.0
HOVER_TIME = 3.0
RATE_HZ = 50.0
LOOKAHEAD_DT = 0.05
CIRCLE_RADIUS = 1.0
DEBUG_LIFT_HEIGHT = HEIGHT
DEBUG_LIFT_DURATION = 10.0
DEBUG_MAX_EXTRA_HEIGHT = 0.20
TAKEOFF_HEIGHT_TOLERANCE = 0.05
TAKEOFF_SETTLE_TIME = 0.5
TAKEOFF_WAIT_TIMEOUT = 8.0
SINGLE_SETPOINT_RAMP_DURATION = 5.0

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
MOCAP_TIMEOUT = 5.0
INITIAL_POSITION_WARN_ERROR = 0.20
ESTIMATOR_POSITION_MAX_ERROR = 0.30
EIGHT_DEBUG_NAMES = ('cf1', 'cf2', 'cf3', 'cf4', 'cf5', 'cf6', 'cf7', 'cf8')
EIGHT_DEBUG_ACTIVE_NAME = 'cf2'


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


def takeoff_and_land(swarm, cfs, time_helper):
    log_initial_position_check(swarm, cfs, wait_for_mocap_poses(swarm, cfs, time_helper))
    require_launch_confirmation(swarm)
    wait_for_poses(cfs, time_helper)
    for cf in cfs:
        cf.takeoff(targetHeight=HEIGHT, duration=TAKEOFF_DURATION)
    time_helper.sleep(TAKEOFF_DURATION + HOVER_TIME)
    for cf in cfs:
        cf.land(targetHeight=LAND_HEIGHT, duration=LAND_DURATION)
    time_helper.sleep(LAND_DURATION + 1.0)


def eight_debug_takeoff(swarm, cfs):
    require_count(cfs, 8)
    time_helper = swarm.timeHelper
    mocap_subscription = None

    try:
        log_initial_position_check(swarm, cfs, wait_for_mocap_poses(swarm, cfs, time_helper))
        require_launch_confirmation(swarm)
        mocap_positions, mocap_subscription = create_mocap_position_monitor(swarm, cfs)
        wait_for_mocap_poses(swarm, cfs, time_helper)
        wait_for_poses(cfs, time_helper)
        require_estimator_matches_mocap(swarm, cfs, mocap_positions)
        debug_takeoff_one(
            swarm,
            next(cf for cf in cfs if cf.prefix.lstrip('/') == 'cf2'),
            time_helper,
            mocap_positions,
        )
    finally:
        if mocap_subscription is not None:
            swarm.allcfs.destroy_subscription(mocap_subscription)


def eight_debug_single_takeoff(swarm, cfs):
    require_count(cfs, 1)
    time_helper = swarm.timeHelper
    mocap_subscription = None
    cf = cfs[0]
    cf_name = cf.prefix.lstrip('/')

    if cf_name != EIGHT_DEBUG_ACTIVE_NAME:
        raise RuntimeError(f'Expected active Crazyflie {EIGHT_DEBUG_ACTIVE_NAME}, found {cf_name}.')

    try:
        mocap_positions = wait_for_mocap_pose_names(swarm, EIGHT_DEBUG_NAMES, time_helper)
        log_initial_position_names(swarm, EIGHT_DEBUG_NAMES, mocap_positions)
        require_launch_confirmation(swarm)
        mocap_positions, mocap_subscription = create_mocap_position_monitor_names(
            swarm, EIGHT_DEBUG_NAMES
        )
        wait_for_mocap_pose_names(swarm, EIGHT_DEBUG_NAMES, time_helper)
        wait_for_poses(cfs, time_helper)
        require_estimator_matches_mocap(swarm, cfs, mocap_positions)
        debug_takeoff_one(swarm, cf, time_helper, mocap_positions)
    finally:
        if mocap_subscription is not None:
            swarm.allcfs.destroy_subscription(mocap_subscription)


def eight_debug_sequence(swarm, cfs):
    require_count(cfs, 8)
    time_helper = swarm.timeHelper
    mocap_subscription = None

    try:
        log_initial_position_check(swarm, cfs, wait_for_mocap_poses(swarm, cfs, time_helper))
        require_launch_confirmation(swarm)
        mocap_positions, mocap_subscription = create_mocap_position_monitor(swarm, cfs)
        wait_for_mocap_poses(swarm, cfs, time_helper)
        wait_for_poses(cfs, time_helper)
        require_estimator_matches_mocap(swarm, cfs, mocap_positions)
        for cf in cfs:
            debug_takeoff_one(swarm, cf, time_helper, mocap_positions)
    finally:
        if mocap_subscription is not None:
            swarm.allcfs.destroy_subscription(mocap_subscription)


def debug_takeoff_one(swarm, cf, time_helper, mocap_positions):
    cf_name = cf.prefix.lstrip('/')
    emergency_stop = False
    start = np.array(mocap_positions[cf_name], dtype=float)
    target = np.array([start[0], start[1], HEIGHT], dtype=float)
    land_target = np.array([start[0], start[1], LAND_HEIGHT], dtype=float)
    max_z = HEIGHT + DEBUG_MAX_EXTRA_HEIGHT

    try:
        swarm.allcfs.get_logger().info(f'Debug takeoff active drone: {cf.prefix}')
        swarm.allcfs.get_logger().info(
            f'Debug streaming target for {cf.prefix}: z={target[2]:.3f} m'
        )
        log_status(swarm, cf, 'before arm')
        require_unlocked(swarm, cf)
        cf.arm(True)
        time_helper.sleep(0.5)
        log_status(swarm, cf, 'after arm')

        stream_position_ramp(
            cf,
            time_helper,
            mocap_positions,
            cf_name,
            start,
            target,
            TAKEOFF_DURATION,
            max_z,
        )
        stream_position_hold(cf, time_helper, mocap_positions, cf_name, target, HOVER_TIME, max_z)
        stream_position_ramp(
            cf,
            time_helper,
            mocap_positions,
            cf_name,
            target,
            land_target,
            LAND_DURATION,
            max_z,
        )
    except KeyboardInterrupt:
        cf.emergency()
        emergency_stop = True
        raise
    except RuntimeError as exc:
        if 'exceeded debug altitude limit' in str(exc):
            emergency_stop = True
        raise
    finally:
        if not emergency_stop:
            notify_setpoints_stop([cf])
            time_helper.sleep(0.2)
            log_status(swarm, cf, 'after land')
            cf.arm(False)


def stream_position_ramp(cf, time_helper, mocap_positions, cf_name, start, target, duration, max_z):
    start_time = time_helper.time()
    while time_helper.time() - start_time < duration:
        guard_debug_altitude(cf, mocap_positions, cf_name, start, max_z)
        alpha = np.clip((time_helper.time() - start_time) / duration, 0.0, 1.0)
        cf.cmdPosition(start + alpha * (target - start), 0.0)
        time_helper.sleepForRate(RATE_HZ)
    cf.cmdPosition(target, 0.0)


def stream_position_hold(cf, time_helper, mocap_positions, cf_name, target, duration, max_z):
    end_time = time_helper.time() + duration
    while time_helper.time() < end_time:
        guard_debug_altitude(cf, mocap_positions, cf_name, target, max_z)
        cf.cmdPosition(target, 0.0)
        time_helper.sleepForRate(RATE_HZ)


def guard_debug_altitude(cf, mocap_positions, cf_name, fallback, max_z):
    current_z = mocap_positions.get(cf_name, fallback)[2]
    if current_z > max_z:
        cf.emergency()
        raise RuntimeError(f'{cf.prefix} exceeded debug altitude limit.')


def wait_for_poses(cfs, time_helper, timeout=POSE_TIMEOUT):
    start = time_helper.time()
    while time_helper.time() - start < timeout:
        if all(cf.poseStamped for cf in cfs):
            return
        time_helper.sleepForRate(20.0)
    missing = [cf.prefix for cf in cfs if not cf.poseStamped]
    raise RuntimeError(f'Missing pose updates for: {", ".join(missing)}')


def wait_for_mocap_poses(swarm, cfs, time_helper, timeout=MOCAP_TIMEOUT):
    return wait_for_mocap_pose_names(
        swarm, [cf.prefix.lstrip('/') for cf in cfs], time_helper, timeout
    )


def wait_for_mocap_pose_names(swarm, names, time_helper, timeout=MOCAP_TIMEOUT):
    if swarm.allcfs.get_parameter('use_sim_time').value:
        return {}

    expected = set(names)
    received = set()
    positions = {}

    def poses_callback(msg):
        update_mocap_positions(msg, expected, received, positions)

    subscription = swarm.allcfs.create_subscription(
        NamedPoseArray, '/poses', poses_callback, mocap_qos_profile()
    )

    start = time_helper.time()
    while time_helper.time() - start < timeout:
        if expected.issubset(received):
            swarm.allcfs.destroy_subscription(subscription)
            return positions
        time_helper.sleepForRate(20.0)

    swarm.allcfs.destroy_subscription(subscription)
    missing = sorted(expected - received)
    raise RuntimeError(f'Missing mocap /poses updates for: {", ".join(missing)}')


def configured_initial_positions():
    config_path = (
        Path(get_package_share_directory('crazyflie_vicon_bringup'))
        / 'config'
        / 'crazyflies_8.yaml'
    )
    with config_path.open('r') as config_file:
        config = yaml.safe_load(config_file)
    return {
        name: np.array(robot['initial_position'], dtype=float)
        for name, robot in config['robots'].items()
        if robot.get('enabled', False)
    }


def log_initial_position_check(swarm, cfs, mocap_positions):
    log_initial_position_names(
        swarm, [cf.prefix.lstrip('/') for cf in cfs], mocap_positions
    )


def log_initial_position_names(swarm, names, mocap_positions):
    if swarm.allcfs.get_parameter('use_sim_time').value:
        return

    logger = swarm.allcfs.get_logger()
    configured = configured_initial_positions()
    logger.info('Preflight initial position check against crazyflies_8.yaml:')
    for name in names:
        expected = configured[name]
        measured = mocap_positions[name]
        error = np.linalg.norm(measured - expected)
        message = (
            f'{name}: mocap=[{measured[0]:+.3f}, {measured[1]:+.3f}, {measured[2]:+.3f}] '
            f'yaml=[{expected[0]:+.3f}, {expected[1]:+.3f}, {expected[2]:+.3f}] '
            f'error={error:.3f} m'
        )
        if error > INITIAL_POSITION_WARN_ERROR:
            logger.warn(message)
        else:
            logger.info(message)


def require_launch_confirmation(swarm):
    if swarm.allcfs.get_parameter('use_sim_time').value:
        return

    response = input("Type 'launch' to continue, or anything else to abort: ")
    if response.strip().lower() != 'launch':
        raise RuntimeError('Launch aborted by user.')


def log_status(swarm, cf, label):
    status = cf.get_status()
    supervisor = int(status.get('supervisor', 0))
    flags = [
        ('can_arm', 1),
        ('is_armed', 2),
        ('auto_arm', 4),
        ('can_fly', 8),
        ('is_flying', 16),
        ('is_tumbled', 32),
        ('is_locked', 64),
    ]
    active = [name for name, bit in flags if supervisor & bit]
    swarm.allcfs.get_logger().info(
        f'{cf.prefix} status {label}: supervisor={supervisor} '
        f'flags={active} battery={status.get("battery", 0.0):.2f}V '
        f'pm_state={status.get("pm_state", 0)} rssi={status.get("rssi", 0)}'
    )


def require_unlocked(swarm, cf):
    supervisor = int(cf.get_status().get('supervisor', 0))
    if supervisor & 64:
        raise RuntimeError(f'{cf.prefix} is locked. Power-cycle or reboot it before flight.')


def require_can_fly(swarm, cf):
    status = cf.get_status()
    supervisor = int(status.get('supervisor', 0))
    battery = float(status.get('battery', 0.0))
    if not supervisor & 8 or battery <= 0.0:
        raise RuntimeError(
            f'{cf.prefix} is not ready to fly: supervisor={supervisor}, battery={battery:.2f}V.'
        )


def require_estimator_matches_mocap(swarm, cfs, mocap_positions):
    if swarm.allcfs.get_parameter('use_sim_time').value:
        return

    logger = swarm.allcfs.get_logger()
    logger.info('Preflight estimator check against mocap:')
    failures = []
    for cf in cfs:
        name = cf.prefix.lstrip('/')
        estimator = np.array(cf.get_position(), dtype=float)
        mocap = mocap_positions[name]
        error = np.linalg.norm(estimator - mocap)
        message = (
            f'{name}: estimator=[{estimator[0]:+.3f}, {estimator[1]:+.3f}, {estimator[2]:+.3f}] '
            f'mocap=[{mocap[0]:+.3f}, {mocap[1]:+.3f}, {mocap[2]:+.3f}] '
            f'error={error:.3f} m'
        )
        if error > ESTIMATOR_POSITION_MAX_ERROR:
            logger.warn(message)
            failures.append(name)
        else:
            logger.info(message)

    if failures:
        raise RuntimeError(
            'Estimator/mocap mismatch too large for: ' + ', '.join(failures)
        )


def mocap_qos_profile():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        deadline=Duration(seconds=0, nanoseconds=10_000_000),
    )


def update_mocap_positions(msg, expected, received, positions):
    for pose in msg.poses:
        if pose.name in expected:
            received.add(pose.name)
            positions[pose.name] = np.array(
                [
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                ],
                dtype=float,
            )


def create_mocap_position_monitor(swarm, cfs):
    return create_mocap_position_monitor_names(
        swarm, [cf.prefix.lstrip('/') for cf in cfs]
    )


def create_mocap_position_monitor_names(swarm, names):
    expected = set(names)
    received = set()
    positions = {}

    def poses_callback(msg):
        update_mocap_positions(msg, expected, received, positions)

    subscription = swarm.allcfs.create_subscription(
        NamedPoseArray, '/poses', poses_callback, mocap_qos_profile()
    )
    return positions, subscription


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


def is_tumbled(cf):
    return bool(int(cf.get_status().get('supervisor', 0)) & 32)


def notify_setpoints_stop(cfs):
    for cf in cfs:
        cf.notifySetpointsStop()


def land_all(cfs, time_helper, swarm=None, mocap_positions=None):
    for cf in cfs:
        cf.land(targetHeight=LAND_HEIGHT, duration=LAND_DURATION)

    if swarm is None:
        time_helper.sleep(LAND_DURATION + 1.0)
        return

    start = time_helper.time()
    last_log_time = start - 1.0
    while time_helper.time() - start < LAND_DURATION + 1.0:
        now = time_helper.time()
        if now - last_log_time >= 1.0:
            for cf in cfs:
                estimator = cf.get_position()
                status = cf.get_status()
                z = measured_height(swarm, cf, mocap_positions or {})
                swarm.allcfs.get_logger().info(
                    f'landing monitor {cf.prefix}: '
                    f'estimator_pos=[{estimator[0]:+.3f}, {estimator[1]:+.3f}, {estimator[2]:+.3f}], '
                    f'mocap_z={z}, '
                    f'supervisor={int(status.get("supervisor", 0))}, '
                    f'battery={status.get("battery", 0.0):.2f}V'
                )
            last_log_time = now
        time_helper.sleepForRate(20.0)


def stream_land_all(cfs, time_helper, swarm, mocap_positions):
    starts = current_positions(cfs)
    targets = starts.copy()
    targets[:, 2] = LAND_HEIGHT
    start_time = time_helper.time()
    last_log_time = start_time - 1.0

    while time_helper.time() - start_time < LAND_DURATION:
        now = time_helper.time()
        alpha = np.clip((now - start_time) / LAND_DURATION, 0.0, 1.0)
        refs = starts + alpha * (targets - starts)
        for cf, ref in zip(cfs, refs):
            if is_tumbled(cf):
                cf.emergency()
                raise RuntimeError(f'{cf.prefix} tumbled during streamed landing.')
            cf.cmdFullState(ref, np.zeros(3), np.zeros(3), 0.0, np.zeros(3))

        if now - last_log_time >= 1.0:
            for cf, ref in zip(cfs, refs):
                estimator = cf.get_position()
                status = cf.get_status()
                z = measured_height(swarm, cf, mocap_positions or {})
                swarm.allcfs.get_logger().info(
                    f'streamed landing monitor {cf.prefix}: '
                    f'estimator_pos=[{estimator[0]:+.3f}, {estimator[1]:+.3f}, {estimator[2]:+.3f}], '
                    f'mocap_z={z}, '
                    f'pos_ref=[{ref[0]:+.3f}, {ref[1]:+.3f}, {ref[2]:+.3f}], '
                    f'supervisor={int(status.get("supervisor", 0))}, '
                    f'battery={status.get("battery", 0.0):.2f}V'
                )
            last_log_time = now

        time_helper.sleepForRate(RATE_HZ)

    notify_setpoints_stop(cfs)
    time_helper.sleep(0.2)
    for cf in cfs:
        cf.arm(False)


def stream_setpoints(
    cfs,
    time_helper,
    goals,
    policy,
    max_time,
    swarm=None,
    mocap_positions=None,
    use_goal_position_ref=False,
    goal_start_positions=None,
    goal_ramp_duration=None,
    min_stream_time=0.0,
):
    wait_for_poses(cfs, time_helper)
    last_positions = current_positions(cfs)
    velocities = np.zeros_like(last_positions)
    start = time_helper.time()
    last_time = start
    last_log_time = start
    logged_first_setpoint = False

    while time_helper.time() - start < max_time:
        now = time_helper.time()
        positions = current_positions(cfs)
        sample_dt = max(now - last_time, 1.0 / RATE_HZ)
        velocities = (positions - last_positions) / sample_dt
        if goal_start_positions is not None and goal_ramp_duration is not None:
            alpha = np.clip((now - start) / goal_ramp_duration, 0.0, 1.0)
            active_goals = goal_start_positions + alpha * (goals - goal_start_positions)
        else:
            alpha = 1.0
            active_goals = goals

        if poses_are_stale(cfs, now):
            raise RuntimeError('Pose data became stale.')
        for cf in cfs:
            if is_tumbled(cf):
                cf.emergency()
                raise RuntimeError(f'{cf.prefix} tumbled during cmdFullState stream.')
        if mocap_positions is not None:
            max_z = HEIGHT + DEBUG_MAX_EXTRA_HEIGHT
            for cf in cfs:
                z = measured_height(swarm, cf, mocap_positions)
                if z is not None and z > max_z:
                    cf.emergency()
                    raise RuntimeError(f'{cf.prefix} exceeded stream altitude limit.')
        if len(cfs) > 1 and pairwise_too_close(positions, SAFE_RADIUS):
            raise RuntimeError('Pairwise distance dropped below safe radius.')
        if (
            now - start >= min_stream_time
            and alpha >= 1.0
            and np.all(np.linalg.norm(goals - positions, axis=1) < GOAL_TOLERANCE)
        ):
            break

        accelerations = policy.accelerations(positions, velocities, active_goals)
        velocity_refs = velocities + accelerations * LOOKAHEAD_DT
        if use_goal_position_ref:
            position_refs = active_goals.copy()
        else:
            position_refs = (
                positions
                + velocities * LOOKAHEAD_DT
                + 0.5 * accelerations * LOOKAHEAD_DT**2
            )

        for cf, pos_ref, vel_ref, acc in zip(cfs, position_refs, velocity_refs, accelerations):
            if not logged_first_setpoint and swarm is not None:
                z = measured_height(swarm, cf, mocap_positions or {})
                swarm.allcfs.get_logger().info(
                    f'First cmdFullState for {cf.prefix}: '
                    f'estimator_pos=[{positions[0, 0]:+.3f}, {positions[0, 1]:+.3f}, {positions[0, 2]:+.3f}], '
                    f'mocap_z={z}, '
                    f'pos_ref=[{pos_ref[0]:+.3f}, {pos_ref[1]:+.3f}, {pos_ref[2]:+.3f}], '
                    f'vel_ref=[{vel_ref[0]:+.3f}, {vel_ref[1]:+.3f}, {vel_ref[2]:+.3f}], '
                    f'acc=[{acc[0]:+.3f}, {acc[1]:+.3f}, {acc[2]:+.3f}]'
                )
            cf.cmdFullState(pos_ref, vel_ref, acc, 0.0, np.zeros(3))
        logged_first_setpoint = True

        if swarm is not None and now - last_log_time >= 1.0:
            for cf, position, goal, pos_ref, vel_ref, acc in zip(
                cfs, positions, goals, position_refs, velocity_refs, accelerations
            ):
                z = measured_height(swarm, cf, mocap_positions or {})
                swarm.allcfs.get_logger().info(
                    f'cmdFullState monitor {cf.prefix}: '
                    f'estimator_pos=[{position[0]:+.3f}, {position[1]:+.3f}, {position[2]:+.3f}], '
                    f'mocap_z={z}, '
                    f'ramp_alpha={alpha:.2f}, '
                    f'goal=[{goal[0]:+.3f}, {goal[1]:+.3f}, {goal[2]:+.3f}], '
                    f'pos_ref=[{pos_ref[0]:+.3f}, {pos_ref[1]:+.3f}, {pos_ref[2]:+.3f}], '
                    f'vel_ref=[{vel_ref[0]:+.3f}, {vel_ref[1]:+.3f}, {vel_ref[2]:+.3f}], '
                    f'acc=[{acc[0]:+.3f}, {acc[1]:+.3f}, {acc[2]:+.3f}]'
                )
            last_log_time = now

        last_positions = positions
        last_time = now
        time_helper.sleepForRate(RATE_HZ)


def measured_height(swarm, cf, mocap_positions):
    if swarm.allcfs.get_parameter('use_sim_time').value:
        return float(cf.get_position()[2])
    position = mocap_positions.get(cf.prefix.lstrip('/'))
    if position is None:
        return None
    return float(position[2])


def wait_for_takeoff_height(swarm, cf, time_helper, mocap_positions):
    logger = swarm.allcfs.get_logger()
    target_z = HEIGHT
    max_z = HEIGHT + DEBUG_MAX_EXTRA_HEIGHT
    start = time_helper.time()
    last_log_time = start
    settled_start = None

    logger.info(f'Waiting for {cf.prefix} to reach takeoff height before cmdFullState.')
    while time_helper.time() - start < TAKEOFF_WAIT_TIMEOUT:
        now = time_helper.time()
        z = measured_height(swarm, cf, mocap_positions)
        if z is None:
            time_helper.sleepForRate(20.0)
            continue
        if now - last_log_time >= 1.0:
            estimator = cf.get_position()
            status = cf.get_status()
            supervisor = int(status.get('supervisor', 0))
            logger.info(
                f'takeoff monitor {cf.prefix}: '
                f'estimator_pos=[{estimator[0]:+.3f}, {estimator[1]:+.3f}, {estimator[2]:+.3f}], '
                f'mocap_z={z:.3f}, '
                f'target_z={target_z:.3f}, '
                f'supervisor={supervisor}, '
                f'battery={status.get("battery", 0.0):.2f}V'
            )
            last_log_time = now
        if z > max_z:
            cf.emergency()
            raise RuntimeError(f'{cf.prefix} exceeded takeoff altitude limit before cmdFullState.')

        if abs(z - target_z) <= TAKEOFF_HEIGHT_TOLERANCE:
            if settled_start is None:
                settled_start = now
            if now - settled_start >= TAKEOFF_SETTLE_TIME:
                logger.info(
                    f'{cf.prefix} reached takeoff height z={z:.3f} m; switching to cmdFullState.'
                )
                return
        else:
            settled_start = None

        time_helper.sleepForRate(20.0)

    raise RuntimeError(f'{cf.prefix} did not reach takeoff height before timeout.')


def single_setpoint(swarm, cfs):
    require_count(cfs, 1)
    time_helper = swarm.timeHelper
    cf = cfs[0]
    mocap_subscription = None
    emergency_stop = False
    low_level_started = False

    model_path = default_model_path()
    policy = GcbfPolicy(model_path, swarm.allcfs.get_logger())

    try:
        log_initial_position_check(swarm, cfs, wait_for_mocap_poses(swarm, cfs, time_helper))
        require_launch_confirmation(swarm)
        mocap_positions, mocap_subscription = create_mocap_position_monitor(swarm, cfs)
        wait_for_poses(cfs, time_helper)
        log_status(swarm, cf, 'before takeoff')
        require_unlocked(swarm, cf)
        cf.takeoff(targetHeight=HEIGHT, duration=TAKEOFF_DURATION)
        wait_for_takeoff_height(swarm, cf, time_helper, mocap_positions)
        start = current_positions(cfs)
        goal = start.copy()
        goal[0] = start[0] + np.array([0.4, 0.0, 0.0])
        goal[0, 2] = HEIGHT
        low_level_started = True
        stream_setpoints(
            cfs,
            time_helper,
            goal,
            policy,
            MAX_TRACKING_TIME,
            swarm=swarm,
            mocap_positions=mocap_positions,
            use_goal_position_ref=True,
            goal_start_positions=start,
            goal_ramp_duration=SINGLE_SETPOINT_RAMP_DURATION,
        )
    except RuntimeError as exc:
        if (
            'exceeded takeoff altitude limit' in str(exc)
            or 'exceeded stream altitude limit' in str(exc)
            or 'tumbled during cmdFullState stream' in str(exc)
        ):
            emergency_stop = True
        raise
    finally:
        if not emergency_stop and low_level_started:
            stream_land_all(cfs, time_helper, swarm, mocap_positions)
        elif not emergency_stop:
            notify_setpoints_stop(cfs)
            time_helper.sleep(0.2)
            land_all(cfs, time_helper, swarm=swarm, mocap_positions=mocap_positions)
        if mocap_subscription is not None:
            swarm.allcfs.destroy_subscription(mocap_subscription)


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
        log_initial_position_check(swarm, cfs, wait_for_mocap_poses(swarm, cfs, time_helper))
        require_launch_confirmation(swarm)
        wait_for_poses(cfs, time_helper)
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
            'eight_debug_takeoff',
            'eight_debug_single_takeoff',
            'eight_debug_sequence',
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
            takeoff_and_land(swarm, cfs, time_helper)
        elif args.mode == 'triple_takeoff':
            require_count(cfs, 3)
            takeoff_and_land(swarm, cfs, time_helper)
        elif args.mode == 'eight_takeoff':
            require_count(cfs, 8)
            takeoff_and_land(swarm, cfs, time_helper)
        elif args.mode == 'eight_debug_takeoff':
            eight_debug_takeoff(swarm, cfs)
        elif args.mode == 'eight_debug_single_takeoff':
            eight_debug_single_takeoff(swarm, cfs)
        elif args.mode == 'eight_debug_sequence':
            eight_debug_sequence(swarm, cfs)
        elif args.mode == 'single_setpoint':
            single_setpoint(swarm, cfs)
        elif args.mode == 'gcbf8_position_exchange':
            gcbf8_position_exchange(swarm, cfs)
    except KeyboardInterrupt:
        if args.mode not in (
            'eight_debug_takeoff',
            'eight_debug_single_takeoff',
            'eight_debug_sequence',
        ):
            notify_setpoints_stop(cfs)
            land_all(cfs, time_helper)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
