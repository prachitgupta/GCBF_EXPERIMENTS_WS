#!/usr/bin/env python3

import rclpy

from crazyflie_py import Crazyswarm

from crazyflie_vicon_bringup.four_drone_setpoint import (
    require_names,
    stream_land_available,
)
from crazyflie_vicon_bringup.gcbf_experiments import (
    create_mocap_position_monitor,
    log_initial_position_check,
    log_status,
    notify_setpoints_stop,
    require_can_fly,
    require_launch_confirmation,
    require_unlocked,
    sorted_crazyflies,
    stream_circular_position_exchange,
    stream_takeoff_all,
    wait_for_mocap_poses,
    wait_for_poses,
    wait_for_status,
)


EXPECTED_NAMES = ('cf1', 'cf2', 'cf3', 'cf4', 'cf5', 'cf6', 'cf7', 'cf8')


def main():
    print(
        'Starting streamed 8-drone position exchange; waiting for Crazyflie ROS services...',
        flush=True,
    )
    swarm = Crazyswarm()
    cfs = sorted_crazyflies(swarm)
    print(
        'Found active Crazyflies: '
        + ', '.join(cf.prefix.lstrip('/') for cf in cfs),
        flush=True,
    )
    require_names(cfs, EXPECTED_NAMES)

    time_helper = swarm.timeHelper
    mocap_subscription = None
    mocap_positions = None
    low_level_started = False

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
        stream_circular_position_exchange(cfs, time_helper, swarm, mocap_positions)
    except RuntimeError as exc:
        swarm.allcfs.get_logger().error(str(exc))
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


if __name__ == '__main__':
    main()
