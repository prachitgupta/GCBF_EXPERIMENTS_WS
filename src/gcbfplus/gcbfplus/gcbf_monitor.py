from pathlib import Path
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from gcbfplus.crazyswarm_double_integrator import (
    DEFAULT_HOVER_HEIGHT,
    default_crazyflies_yaml,
    pairwise_min_distance,
    plot_pose_errors,
    save_position_animation,
    save_results,
)
from gcbfplus.gcbf_state_bridge import N_AGENTS, STATE_WIDTH


class GcbfMonitor(Node):
    def __init__(self):
        super().__init__("gcbf_monitor")
        self.declare_parameter("crazyflies_yaml", str(default_crazyflies_yaml()))
        self.declare_parameter("output", "src/gcbfplus/media/results/gcbf_crazyswarm_double_integrator.npz")
        self.declare_parameter("save_error_plots", False)
        self.declare_parameter("error_plot_dir", "src/gcbfplus/media/results/pose_errors")
        self.declare_parameter("save_animation", False)
        self.declare_parameter("animation_file", "src/gcbfplus/media/results/gcbf_crazyswarm_double_integrator.gif")
        self.declare_parameter("animation_stride", 5)
        self.declare_parameter("hover_height", DEFAULT_HOVER_HEIGHT)

        from gcbfplus.crazyswarm_double_integrator import load_crazyflies

        robot_items = load_crazyflies(Path(self.get_parameter("crazyflies_yaml").value))
        self.robot_names = [name for name, _ in robot_items]

        self.latest_action = None
        self.latest_cbf = None
        self.start_time = None
        self.results = {
            "time": [],
            "positions": [],
            "velocities": [],
            "actions": [],
            "goals": None,
            "robot_names": self.robot_names,
            "min_distance": [],
            "cbf": [],
        }

        self.create_subscription(Float64MultiArray, "/gcbf/state", self.state_callback, 50)
        self.create_subscription(Float64MultiArray, "/gcbf/action", self.action_callback, 50)
        self.create_subscription(Float64MultiArray, "/gcbf/cbf", self.cbf_callback, 50)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def action_callback(self, msg):
        data = np.asarray(msg.data, dtype=float)
        if data.size == N_AGENTS * 2:
            self.latest_action = data.reshape(N_AGENTS, 2)

    def cbf_callback(self, msg):
        data = np.asarray(msg.data, dtype=float)
        if data.size == N_AGENTS:
            self.latest_cbf = data.reshape(N_AGENTS)

    def state_callback(self, msg):
        data = np.asarray(msg.data, dtype=float)
        if data.size != N_AGENTS * STATE_WIDTH:
            return

        now = self.now_seconds()
        if self.start_time is None:
            self.start_time = now
        state = data.reshape(N_AGENTS, STATE_WIDTH)
        positions = state[:, 0:3]
        velocities_xy = state[:, 3:5]
        goals = state[:, 6:8]
        if self.results["goals"] is None:
            self.results["goals"] = goals.copy()

        self.results["time"].append(now - self.start_time)
        self.results["positions"].append(positions.copy())
        self.results["velocities"].append(velocities_xy.copy())
        self.results["actions"].append(
            np.zeros((N_AGENTS, 2), dtype=float) if self.latest_action is None else self.latest_action.copy()
        )
        self.results["min_distance"].append(pairwise_min_distance(positions[:, :2]))
        self.results["cbf"].append(
            np.zeros(N_AGENTS, dtype=float) if self.latest_cbf is None else self.latest_cbf.copy()
        )

    def save(self):
        output = Path(self.get_parameter("output").value).expanduser().resolve()
        if self.results["goals"] is None:
            self.get_logger().warn("No /gcbf/state samples received; nothing to save.")
            return
        save_results(output, self.results)
        self.get_logger().info(f"Saved results: {output}")

        if bool(self.get_parameter("save_error_plots").value):
            plot_pose_errors(
                self.results,
                Path(self.get_parameter("error_plot_dir").value).expanduser().resolve(),
                float(self.get_parameter("hover_height").value),
            )
        if bool(self.get_parameter("save_animation").value):
            save_position_animation(
                self.results,
                Path(self.get_parameter("animation_file").value).expanduser().resolve(),
                0.05,
                int(self.get_parameter("animation_stride").value),
            )


def main(args=None):
    rclpy.init(args=args)
    node = GcbfMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
