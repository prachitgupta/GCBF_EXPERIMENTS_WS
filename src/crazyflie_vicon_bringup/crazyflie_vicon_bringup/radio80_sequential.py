"""Sequential independent-control test for one or two configured Crazyflies."""

from crazyflie_py import Crazyswarm
import numpy as np


HEIGHT = 0.4
TAKEOFF_DURATION = 2.5
MOVE_DURATION = 2.0
HOVER_DURATION = 2.0
LAND_DURATION = 2.5


def fly_slot(cf, time_helper, y_offset):
    """Fly and land one Crazyflie while all other Crazyflies remain landed."""
    cf.takeoff(targetHeight=HEIGHT, duration=TAKEOFF_DURATION)
    time_helper.sleep(TAKEOFF_DURATION + HOVER_DURATION)

    goal = np.array(cf.initialPosition) + np.array([0.0, y_offset, HEIGHT])
    cf.goTo(goal, yaw=0.0, duration=MOVE_DURATION)
    time_helper.sleep(MOVE_DURATION + HOVER_DURATION)

    cf.land(targetHeight=0.04, duration=LAND_DURATION)
    time_helper.sleep(LAND_DURATION)


def main():
    """Run the one- or two-Crazyflie sequential flight sequence."""
    swarm = Crazyswarm()
    time_helper = swarm.timeHelper
    crazyflies = swarm.allcfs.crazyflies
    logger = swarm.allcfs.get_logger()

    if len(crazyflies) not in (1, 2):
        raise RuntimeError(
            'This experiment requires one or two configured Crazyflies.'
        )

    if len(crazyflies) == 1:
        logger.info(
            'Single-Crazyflie rehearsal: one vehicle will fly both slots.'
        )
        sequence = [(crazyflies[0], 0.25), (crazyflies[0], -0.25)]
    else:
        logger.info(
            'Dual-Crazyflie test: each vehicle will fly independently.'
        )
        sequence = [(crazyflies[0], 0.25), (crazyflies[1], -0.25)]

    for index, (cf, y_offset) in enumerate(sequence, start=1):
        logger.info(f'Starting slot {index} with {cf.prefix}.')
        fly_slot(cf, time_helper, y_offset)

    logger.info('Sequential radio 80 experiment complete.')


if __name__ == '__main__':
    main()
