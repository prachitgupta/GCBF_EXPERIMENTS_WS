#!/usr/bin/env python3

import os
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = '88'

from ament_index_python.packages import get_package_share_directory
from motion_capture_tracking_interfaces.msg import NamedPoseArray
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
import yaml


CRAZYFLIE_NAMES = ('cf1', 'cf2', 'cf3', 'cf4', 'cf5', 'cf6', 'cf7', 'cf8')
WARN_ERROR = 0.20


def mocap_qos_profile():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        deadline=Duration(seconds=0, nanoseconds=10_000_000),
    )


def configured_initial_positions():
    config_path = (
        Path(get_package_share_directory('crazyflie_vicon_bringup'))
        / 'config'
        / 'crazyflies_8.yaml'
    )
    with config_path.open('r') as config_file:
        config = yaml.safe_load(config_file)
    return {
        name: np.array(config['robots'][name]['initial_position'], dtype=float)
        for name in CRAZYFLIE_NAMES
    }


class ViconPositionCheck(Node):
    def __init__(self):
        super().__init__('check_vicon_positions')
        self.expected = set(CRAZYFLIE_NAMES)
        self.positions = {}
        self.configured = configured_initial_positions()
        self.create_subscription(
            NamedPoseArray,
            '/poses',
            self.poses_callback,
            mocap_qos_profile(),
        )
        self.get_logger().info('Waiting for /poses entries for cf1 through cf8.')

    def poses_callback(self, msg):
        for pose in msg.poses:
            if pose.name in self.expected:
                self.positions[pose.name] = np.array(
                    [
                        pose.pose.position.x,
                        pose.pose.position.y,
                        pose.pose.position.z,
                    ],
                    dtype=float,
                )

        if self.expected.issubset(self.positions):
            self.report()
            rclpy.shutdown()

    def report(self):
        self.get_logger().info('Vicon position check against crazyflies_8.yaml:')
        for name in CRAZYFLIE_NAMES:
            measured = self.positions[name]
            expected = self.configured[name]
            error = np.linalg.norm(measured - expected)
            message = (
                f'{name}: mocap=[{measured[0]:+.3f}, {measured[1]:+.3f}, {measured[2]:+.3f}] '
                f'yaml=[{expected[0]:+.3f}, {expected[1]:+.3f}, {expected[2]:+.3f}] '
                f'error={error:.3f} m'
            )
            if error > WARN_ERROR:
                self.get_logger().warn(message)
            else:
                self.get_logger().info(message)


def main():
    rclpy.init()
    node = ViconPositionCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
