#!/usr/bin/env python3

import numpy as np
import rclpy

from crazyflie_py import Crazyswarm

from crazyflie_vicon_bringup.gcbf_experiments import (
    GcbfPolicy,
    HEIGHT,
    HOVER_TIME,
    LAND_DURATION,
    LAND_HEIGHT,
    MAX_TRACKING_TIME,
    RATE_HZ,
    SINGLE_SETPOINT_RAMP_DURATION,
    create_mocap_position_monitor,
    current_positions,
    default_model_path,
    log_initial_position_check,
    log_status,
    notify_setpoints_stop,
    require_launch_confirmation,
    require_can_fly,
    require_unlocked,
    is_tumbled,
    measured_height,
    sorted_crazyflies,
    stream_takeoff_all,
    stream_setpoints,
    wait_for_mocap_poses,
    wait_for_poses,
    wait_for_status,
)


FOUR_NAMES = ('cf1', 'cf2', 'cf3', 'cf4')
STREAM_LAND_TOUCHDOWN_HEIGHT = 0.13
STREAM_LAND_TIMEOUT = LAND_DURATION + 3.0
DISARM_SETTLE_TIME = 0.3


def require_names(cfs, expected_names):
    names = tuple(cf.prefix.lstrip('/') for cf in cfs)
    if names != expected_names:
        raise RuntimeError(f'Expected active Crazyflies {expected_names}, found {names}.')


def stop_and_disarm(cf, time_helper):
    cf.notifySetpointsStop(remainValidMillisecs=0)
    time_helper.sleep(0.1)
    cf.arm(False)
    time_helper.sleep(DISARM_SETTLE_TIME)


def stream_land_available(cfs, time_helper, swarm, mocap_positions):
    active = [cf for cf in cfs if not is_tumbled(cf)]
    skipped = [cf.prefix for cf in cfs if is_tumbled(cf)]
    if skipped:
        swarm.allcfs.get_logger().warn(
            'Skipping streamed landing for tumbled Crazyflies: ' + ', '.join(skipped)
        )
    if not active:
        return

    starts = {
        cf.prefix: position
        for cf, position in zip(active, current_positions(active))
    }
    start_time = time_helper.time()
    last_log_time = start_time - 1.0

    while active and time_helper.time() - start_time < STREAM_LAND_TIMEOUT:
        now = time_helper.time()
        alpha = np.clip((now - start_time) / LAND_DURATION, 0.0, 1.0)
        still_active = []
        refs = []

        for cf in active:
            if is_tumbled(cf):
                cf.emergency()
                continue

            z = measured_height(swarm, cf, mocap_positions or {})
            if z is not None and z <= STREAM_LAND_TOUCHDOWN_HEIGHT:
                swarm.allcfs.get_logger().info(
                    f'{cf.prefix} reached touchdown height z={z:.3f} m; stopping streamed landing.'
                )
                stop_and_disarm(cf, time_helper)
                continue

            start = starts[cf.prefix]
            target = start.copy()
            target[2] = LAND_HEIGHT
            ref = start + alpha * (target - start)
            cf.cmdFullState(ref, np.zeros(3), np.zeros(3), 0.0, np.zeros(3))
            still_active.append(cf)
            refs.append(ref)

        active = still_active

        if now - last_log_time >= 1.0:
            for cf, ref in zip(active, refs):
                status = cf.get_status()
                estimator = cf.get_position()
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

    for cf in active:
        swarm.allcfs.get_logger().warn(
            f'{cf.prefix} did not report touchdown before landing timeout; stopping and disarming.'
        )
        stop_and_disarm(cf, time_helper)


def main_for_names(expected_names=FOUR_NAMES):
    print(
        f'Starting streamed hover for {len(expected_names)} Crazyflie(s); '
        'waiting for Crazyflie ROS services...',
        flush=True,
    )
    swarm = Crazyswarm()
    cfs = sorted_crazyflies(swarm)
    print(
        'Found active Crazyflies: '
        + ', '.join(cf.prefix.lstrip('/') for cf in cfs),
        flush=True,
    )
    require_names(cfs, expected_names)

    time_helper = swarm.timeHelper
    mocap_subscription = None
    low_level_started = False
    policy = GcbfPolicy(default_model_path(), swarm.allcfs.get_logger())

    try:
        log_initial_position_check(swarm, cfs, wait_for_mocap_poses(swarm, cfs, time_helper))
        for cf in cfs:
            wait_for_status(swarm, cf, time_helper)
            log_status(swarm, cf, 'preflight radio')
            require_unlocked(swarm, cf)
            require_can_fly(swarm, cf)
        require_launch_confirmation(swarm)
        mocap_positions, mocap_subscription = create_mocap_position_monitor(swarm, cfs)
        wait_for_poses(cfs, time_helper)

        low_level_started = True
        stream_takeoff_all(cfs, time_helper, swarm, mocap_positions)
        start = current_positions(cfs)
        goals = start.copy()
        goals[:, 2] = HEIGHT
        stream_setpoints(
            cfs,
            time_helper,
            goals,
            policy,
            MAX_TRACKING_TIME,
            swarm=swarm,
            mocap_positions=mocap_positions,
            use_goal_position_ref=True,
            goal_start_positions=start,
            goal_ramp_duration=SINGLE_SETPOINT_RAMP_DURATION,
            min_stream_time=HOVER_TIME,
        )
    except RuntimeError as exc:
        if (
            'exceeded takeoff altitude limit' in str(exc)
            or 'exceeded streamed takeoff altitude limit' in str(exc)
            or 'exceeded stream altitude limit' in str(exc)
            or 'tumbled during streamed takeoff' in str(exc)
            or 'tumbled during cmdFullState stream' in str(exc)
            or 'tumbled during streamed landing' in str(exc)
            or 'Pose data became stale' in str(exc)
            or 'Not all Crazyflies reached streamed takeoff height' in str(exc)
        ):
            swarm.allcfs.get_logger().error(str(exc))
        else:
            raise
    finally:
        if low_level_started:
            stream_land_available(cfs, time_helper, swarm, mocap_positions)
        else:
            notify_setpoints_stop(cfs)
        if mocap_subscription is not None:
            swarm.allcfs.destroy_subscription(mocap_subscription)
        if rclpy.ok():
            swarm.allcfs.destroy_node()
            rclpy.shutdown()


def main():
    main_for_names(FOUR_NAMES)


if __name__ == '__main__':
    main()
