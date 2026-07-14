import threading
from pathlib import Path

import jax
import jax.random as jr
import numpy as np
import rclpy
from crazyflie_interfaces.msg import FullState
from crazyflie_interfaces.srv import Takeoff
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from gcbfplus.crazyswarm_double_integrator import (
    DEFAULT_GOAL_TOLERANCE,
    DEFAULT_HOVER_HEIGHT,
    DEFAULT_LOOKAHEAD_DT,
    DEFAULT_MAX_RUNTIME,
    DEFAULT_RATE_HZ,
    N_AGENTS,
    POSE_TIMEOUT,
    TAKEOFF_DURATION,
    default_crazyflies_yaml,
    default_model_dir,
    load_crazyflies,
    make_algo_from_checkpoint,
    make_graph,
    pairwise_min_distance,
)
from gcbfplus.gcbf_state_bridge import STATE_WIDTH


class GcbfActor(Node):
    def __init__(self):
        super().__init__("gcbf_actor")
        self.declare_parameter("crazyflies_yaml", str(default_crazyflies_yaml()))
        self.declare_parameter("model_dir", str(default_model_dir()))
        self.declare_parameter("step", 1000)
        self.declare_parameter("rate_hz", DEFAULT_RATE_HZ)
        self.declare_parameter("lookahead_dt", DEFAULT_LOOKAHEAD_DT)
        self.declare_parameter("hover_height", DEFAULT_HOVER_HEIGHT)
        self.declare_parameter("hover_epsilon", 0.06)
        self.declare_parameter("takeoff_duration", TAKEOFF_DURATION)
        self.declare_parameter("takeoff_timeout", 30.0)
        self.declare_parameter("goal_tolerance", DEFAULT_GOAL_TOLERANCE)
        self.declare_parameter("max_runtime", DEFAULT_MAX_RUNTIME)
        self.declare_parameter("pose_timeout", POSE_TIMEOUT)
        self.declare_parameter("area_size", 4.0)
        self.declare_parameter("seed", 2)

        max_runtime = float(self.get_parameter("max_runtime").value)
        self.env, self.algo = make_algo_from_checkpoint(
            Path(self.get_parameter("model_dir").value),
            int(self.get_parameter("step").value),
            float(self.get_parameter("area_size").value),
            max_runtime,
        )
        self.act_fn = jax.jit(self.algo.act)
        self.cbf_fn = jax.jit(self.algo.get_cbf)
        initial_graph = self.env.reset(jr.PRNGKey(int(self.get_parameter("seed").value)))
        self.obstacle_state = initial_graph.env_states.obstacle
        self.mass = float(self.env._params["m"])
        self.collision_distance = 2.0 * float(self.env._params["car_radius"])
        jax.block_until_ready(self.act_fn(initial_graph))
        jax.block_until_ready(self.cbf_fn(initial_graph))

        robot_items = load_crazyflies(Path(self.get_parameter("crazyflies_yaml").value))
        self.robot_names = [name for name, _ in robot_items]
        self.command_publishers = {
            name: self.create_publisher(FullState, f"/{name}/cmd_full_state", 10)
            for name in self.robot_names
        }
        self.action_pub = self.create_publisher(Float64MultiArray, "/gcbf/action", 10)
        self.cbf_pub = self.create_publisher(Float64MultiArray, "/gcbf/cbf", 10)
        self.create_subscription(Float64MultiArray, "/gcbf/state", self.state_callback, 10)
        self.takeoff_client = self.create_client(Takeoff, "/all/takeoff")

        self.lock = threading.Lock()
        self.latest_state = None
        self.latest_state_time = None
        self.takeoff_requested = False
        self.takeoff_request_time = None
        self.policy_start_time = None
        self.policy_active = False
        self.policy_done = False

        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.create_timer(period, self.control_step)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def state_callback(self, msg):
        data = np.asarray(msg.data, dtype=float)
        if data.size != N_AGENTS * STATE_WIDTH:
            self.get_logger().warn(f"Ignoring /gcbf/state with {data.size} values.")
            return
        with self.lock:
            self.latest_state = data.reshape(N_AGENTS, STATE_WIDTH)
            self.latest_state_time = self.now_seconds()

    def fresh_state(self):
        with self.lock:
            if self.latest_state is None or self.latest_state_time is None:
                return None
            age = self.now_seconds() - self.latest_state_time
            if age > float(self.get_parameter("pose_timeout").value):
                return None
            return self.latest_state.copy()

    def request_takeoff(self):
        if self.takeoff_requested or not self.takeoff_client.service_is_ready():
            return
        request = Takeoff.Request()
        request.group_mask = 0
        request.height = float(self.get_parameter("hover_height").value)
        duration = float(self.get_parameter("takeoff_duration").value)
        request.duration.sec = int(duration)
        request.duration.nanosec = int((duration - int(duration)) * 1e9)
        self.takeoff_client.call_async(request)
        self.takeoff_requested = True
        self.takeoff_request_time = self.now_seconds()
        self.get_logger().info("Requested all-CF takeoff; waiting for hover confirmation.")

    def hover_confirmed(self, state):
        hover_height = float(self.get_parameter("hover_height").value)
        hover_epsilon = float(self.get_parameter("hover_epsilon").value)
        return np.all(np.abs(state[:, 2] - hover_height) <= hover_epsilon)

    def control_step(self):
        if self.policy_done:
            state = self.fresh_state()
            if state is not None:
                hover_height = float(self.get_parameter("hover_height").value)
                for name, row in zip(self.robot_names, state):
                    msg = FullState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "/world"
                    msg.pose.position.x = float(row[0])
                    msg.pose.position.y = float(row[1])
                    msg.pose.position.z = hover_height
                    msg.pose.orientation.w = 1.0
                    self.command_publishers[name].publish(msg)
            return

        state = self.fresh_state()
        if state is None:
            return

        if not self.takeoff_requested:
            self.request_takeoff()
            return

        if not self.policy_active:
            if self.hover_confirmed(state):
                self.policy_active = True
                self.policy_start_time = self.now_seconds()
                self.get_logger().info("All CFs confirmed at hover height; starting GCBF actor.")
            elif self.now_seconds() - self.takeoff_request_time > float(self.get_parameter("takeoff_timeout").value):
                self.get_logger().error("Timed out waiting for hover confirmation.")
                self.policy_done = True
            return

        elapsed = self.now_seconds() - self.policy_start_time
        if elapsed > float(self.get_parameter("max_runtime").value):
            self.get_logger().info("GCBF actor reached max runtime.")
            self.policy_done = True
            return

        positions_xy = state[:, 0:2]
        velocities_xy = state[:, 3:5]
        goals_xy = state[:, 6:8]
        graph = make_graph(self.env, self.obstacle_state, positions_xy, velocities_xy, goals_xy)
        action = np.asarray(self.env.clip_action(self.act_fn(graph)), dtype=float)
        cbf = np.asarray(self.cbf_fn(graph), dtype=float).reshape(N_AGENTS)

        min_distance = pairwise_min_distance(positions_xy)
        if min_distance < self.collision_distance:
            self.get_logger().error(
                f"Pairwise distance {min_distance:.3f} below collision boundary {self.collision_distance:.3f}."
            )
            self.policy_done = True
            return

        self.publish_array(self.action_pub, action, "action_width", 2)
        self.publish_array(self.cbf_pub, cbf.reshape(N_AGENTS, 1), "cbf_width", 1)

        goal_errors = np.linalg.norm(goals_xy - positions_xy, axis=1)
        if np.all(goal_errors < float(self.get_parameter("goal_tolerance").value)):
            self.get_logger().info("All CFs reached position-exchange goals.")
            self.policy_done = True
            return

        accel_xy = action / self.mass
        lookahead_dt = float(self.get_parameter("lookahead_dt").value)
        velocity_refs_xy = velocities_xy + accel_xy * lookahead_dt
        position_refs_xy = positions_xy + velocities_xy * lookahead_dt + 0.5 * accel_xy * lookahead_dt**2
        hover_height = float(self.get_parameter("hover_height").value)

        for name, pos_xy, vel_xy, acc_xy in zip(self.robot_names, position_refs_xy, velocity_refs_xy, accel_xy):
            msg = FullState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "/world"
            msg.pose.position.x = float(pos_xy[0])
            msg.pose.position.y = float(pos_xy[1])
            msg.pose.position.z = hover_height
            msg.pose.orientation.w = 1.0
            msg.twist.linear.x = float(vel_xy[0])
            msg.twist.linear.y = float(vel_xy[1])
            msg.acc.x = float(acc_xy[0])
            msg.acc.y = float(acc_xy[1])
            self.command_publishers[name].publish(msg)

    def publish_array(self, publisher, values, width_label, width):
        msg = Float64MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label="agents", size=N_AGENTS, stride=N_AGENTS * width),
            MultiArrayDimension(label=width_label, size=width, stride=width),
        ]
        msg.data = np.asarray(values, dtype=float).reshape(-1).tolist()
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GcbfActor()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
