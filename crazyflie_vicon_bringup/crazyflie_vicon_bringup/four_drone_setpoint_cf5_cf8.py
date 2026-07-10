#!/usr/bin/env python3

from crazyflie_vicon_bringup.four_drone_setpoint import main_for_names


def main():
    main_for_names(('cf5', 'cf6', 'cf7', 'cf8'))


if __name__ == '__main__':
    main()
