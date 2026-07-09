#!/usr/bin/env python3

import math
import os

os.environ['ROS_DOMAIN_ID'] = '88'

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node


def quat_to_rpy_deg(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class PoseDebug(Node):
    def __init__(self):
        super().__init__('pose_debug')
        self.declare_parameter('topic', '/cf8/pose')
        topic = self.get_parameter('topic').value
        self.create_subscription(PoseStamped, topic, self.pose_callback, 10)
        self.get_logger().info(f'Listening to {topic}')

    def pose_callback(self, msg):
        p = msg.pose.position
        roll, pitch, yaw = quat_to_rpy_deg(msg.pose.orientation)
        self.get_logger().info(
            f'x={p.x:+.3f} m y={p.y:+.3f} m z={p.z:+.3f} m '
            f'roll={roll:+.1f} deg pitch={pitch:+.1f} deg yaw={yaw:+.1f} deg'
        )


def main():
    rclpy.init()
    node = PoseDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
