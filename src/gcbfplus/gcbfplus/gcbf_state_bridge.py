from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from tf2_msgs.msg import TFMessage

from gcbfplus.crazyswarm_double_integrator import (
    N_AGENTS,
    default_crazyflies_yaml,
    load_crazyflies,
)


STATE_WIDTH = 9


class GcbfStateBridge(Node):
    def __init__(self):
        super().__init__("gcbf_state_bridge")
        self.declare_parameter("crazyflies_yaml", str(default_crazyflies_yaml()))
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("pose_timeout", 0.5)

        self.robot_items = load_crazyflies(Path(self.get_parameter("crazyflies_yaml").value))
        self.robot_names = [name for name, _ in self.robot_items]
        starts_xy = np.array([position[:2] for _, position in self.robot_items], dtype=float)
        self.goals_xy = np.roll(starts_xy, shift=N_AGENTS // 2, axis=0)

        self.positions = {}
        self.position_times = {}
        self.last_positions = None
        self.last_publish_time = None

        self.publisher = self.create_publisher(Float64MultiArray, "/gcbf/state", 10)
        self.create_subscription(TFMessage, "/tf", self.tf_callback, 50)
        for name in self.robot_names:
            self.create_subscription(
                PoseStamped,
                f"/{name}/pose",
                lambda msg, robot_name=name: self.pose_callback(msg, robot_name),
                10,
            )

        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.create_timer(period, self.publish_state)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tf_callback(self, msg):
        now = self.now_seconds()
        for transform in msg.transforms:
            child = transform.child_frame_id.lstrip("/")
            parent = transform.header.frame_id.lstrip("/")
            if parent != "world" or child not in self.robot_names:
                continue
            position = transform.transform.translation
            self.positions[child] = np.array([position.x, position.y, position.z], dtype=float)
            self.position_times[child] = now

    def pose_callback(self, msg, robot_name):
        position = msg.pose.position
        self.positions[robot_name] = np.array([position.x, position.y, position.z], dtype=float)
        self.position_times[robot_name] = self.now_seconds()

    def publish_state(self):
        now = self.now_seconds()
        timeout = float(self.get_parameter("pose_timeout").value)
        if any(name not in self.positions for name in self.robot_names):
            return
        if any(now - self.position_times[name] > timeout for name in self.robot_names):
            return

        positions = np.array([self.positions[name] for name in self.robot_names], dtype=float)
        if self.last_positions is None or self.last_publish_time is None:
            velocities = np.zeros_like(positions)
        else:
            dt = max(now - self.last_publish_time, 1e-6)
            velocities = (positions - self.last_positions) / dt

        rows = np.zeros((N_AGENTS, STATE_WIDTH), dtype=float)
        rows[:, 0:3] = positions
        rows[:, 3:6] = velocities
        rows[:, 6:8] = self.goals_xy
        rows[:, 8] = float(self.get_parameter("rate_hz").value)

        msg = Float64MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label="agents", size=N_AGENTS, stride=N_AGENTS * STATE_WIDTH),
            MultiArrayDimension(label="state_width", size=STATE_WIDTH, stride=STATE_WIDTH),
        ]
        msg.data = rows.reshape(-1).tolist()
        self.publisher.publish(msg)

        self.last_positions = positions
        self.last_publish_time = now


def main(args=None):
    rclpy.init(args=args)
    node = GcbfStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
