import threading
import time
from dataclasses import dataclass
from pathlib import Path

import jax
import jax.numpy as jnp
import jax.random as jr
import numpy as np
import rclpy
from crazyflie_interfaces.msg import FullState
from crazyflie_interfaces.srv import Arm
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from tf2_msgs.msg import TFMessage

from gcbfplus.crazyswarm_double_integrator import (
    DEFAULT_GOAL_TOLERANCE,
    DEFAULT_HOVER_HEIGHT,
    DEFAULT_MAX_RUNTIME,
    DEFAULT_RATE_HZ,
    LAND_DURATION,
    LAND_HEIGHT,
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


GOAL_HOLD_SECONDS = 2.0
DEFAULT_LOOKAHEAD_DT = 0.05


@dataclass(frozen=True)
class StateSnapshot:
    sequence: int
    source_time: float
    received_time: float
    state: np.ndarray


class GcbfActor(Node):
    def __init__(self):
        super().__init__("gcbf_actor")
        self.declare_parameter("mode", "sim")
        self.declare_parameter("crazyflies_yaml", str(default_crazyflies_yaml()))
        self.declare_parameter("model_dir", str(default_model_dir()))
        self.declare_parameter("step", 1000)
        self.declare_parameter("rate_hz", DEFAULT_RATE_HZ)
        self.declare_parameter("lookahead_dt", DEFAULT_LOOKAHEAD_DT)
        self.declare_parameter("hover_height", DEFAULT_HOVER_HEIGHT)
        self.declare_parameter("hover_epsilon", 0.06)
        self.declare_parameter("takeoff_duration", TAKEOFF_DURATION)
        self.declare_parameter("takeoff_timeout", 60.0)
        self.declare_parameter("goal_tolerance", DEFAULT_GOAL_TOLERANCE)
        self.declare_parameter("max_runtime", DEFAULT_MAX_RUNTIME)
        self.declare_parameter("pose_timeout", POSE_TIMEOUT)
        self.declare_parameter("max_action_age", 0.02)
        self.declare_parameter("area_size", 4.0)
        self.declare_parameter("seed", 2)
        self.declare_parameter("print_latency", False)

        self.mode = str(self.get_parameter("mode").value).lower()
        if self.mode not in {"sim", "real"}:
            raise ValueError("mode must be 'sim' or 'real'")
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        if self.rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        self.min_period = 1.0 / self.rate_hz

        max_runtime = float(self.get_parameter("max_runtime").value)
        self.env, self.algo = make_algo_from_checkpoint(
            Path(self.get_parameter("model_dir").value),
            int(self.get_parameter("step").value),
            float(self.get_parameter("area_size").value),
            max_runtime,
        )
        initial_graph = self.env.reset(jr.PRNGKey(int(self.get_parameter("seed").value)))
        self.obstacle_state = initial_graph.env_states.obstacle
        self.mass = float(self.env._params["m"])
        self.collision_distance = 2.0 * float(self.env._params["car_radius"])

        robot_items = load_crazyflies(Path(self.get_parameter("crazyflies_yaml").value))
        self.robot_names = [name for name, _ in robot_items]
        if len(self.robot_names) != N_AGENTS:
            raise RuntimeError(f"Expected {N_AGENTS} enabled Crazyflies, found {len(self.robot_names)}")
        starts_xy = np.asarray([position[:2] for _, position in robot_items], dtype=float)
        self.goals_xy = np.roll(starts_xy, shift=N_AGENTS // 2, axis=0)
        self.goals_jax = jnp.asarray(self.goals_xy, dtype=jnp.float32)

        def infer(positions_xy, velocities_xy, goals_xy, obstacle_state):
            graph = make_graph(
                self.env, obstacle_state, positions_xy, velocities_xy, goals_xy
            )
            return self.env.clip_action(self.algo.act(graph)), self.algo.get_cbf(graph)

        self.infer_fn = jax.jit(infer)
        warmup = self.infer_fn(
            jnp.asarray(starts_xy, dtype=jnp.float32),
            jnp.zeros((N_AGENTS, 2), dtype=jnp.float32),
            self.goals_jax,
            self.obstacle_state,
        )
        jax.block_until_ready(warmup)

        self.command_publishers = {
            name: self.create_publisher(FullState, f"/{name}/cmd_full_state", 1)
            for name in self.robot_names
        }
        self.state_pub = self.create_publisher(Float64MultiArray, "/gcbf/state", 1)
        self.action_pub = self.create_publisher(Float64MultiArray, "/gcbf/action", 1)
        self.cbf_pub = self.create_publisher(Float64MultiArray, "/gcbf/cbf", 1)

        self.callback_group = ReentrantCallbackGroup()
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        if self.mode == "sim":
            self.state_subscription = self.create_subscription(
                TFMessage, "/tf", self.tf_callback, sensor_qos,
                callback_group=self.callback_group,
            )
            self.arm_client = None
        else:
            self.state_subscription = self.create_subscription(
                NamedPoseArray, "/poses", self.poses_callback, sensor_qos,
                callback_group=self.callback_group,
            )
            self.arm_client = self.create_client(Arm, "/all/arm")

        self.lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.pending_snapshot = None
        self.inference_running = False
        self.last_cycle_start = -np.inf
        self.sequence = 0
        self.previous_positions = None
        self.previous_source_time = None

        self.phase = "waiting"
        self.phase_start_time = None
        self.takeoff_start_positions = None
        self.policy_start_time = None
        self.hold_positions = None
        self.landing_start_positions = None
        self.arm_requested = False
        self.disarm_requested = False

        self.print_latency = bool(self.get_parameter("print_latency").value)
        self.latency_samples = {}
        self.latency_counts = {
            "processed": 0,
            "replaced": 0,
            "stale": 0,
            "incomplete": 0,
            "deadline_miss": 0,
        }
        self.last_latency_report = time.perf_counter()
        if self.print_latency:
            self.get_logger().info(
                f"JAX backend={jax.default_backend()} devices={jax.devices()}"
            )
        self.get_logger().info(
            f"Single-node {self.mode} controller listening on "
            f"{'/tf' if self.mode == 'sim' else '/poses'} at up to {self.rate_hz:.1f} Hz."
        )

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def stamp_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def tf_callback(self, msg):
        positions = {}
        stamps = []
        for transform in msg.transforms:
            child = transform.child_frame_id.lstrip("/")
            parent = transform.header.frame_id.lstrip("/")
            if parent != "world" or child not in self.robot_names:
                continue
            p = transform.transform.translation
            positions[child] = np.array([p.x, p.y, p.z], dtype=float)
            stamps.append(self.stamp_seconds(transform.header.stamp))
        if len(positions) != N_AGENTS:
            self.latency_counts["incomplete"] += 1
            return
        self.accept_positions(positions, max(stamps) if stamps else 0.0)

    def poses_callback(self, msg):
        positions = {
            item.name: np.array(
                [item.pose.position.x, item.pose.position.y, item.pose.position.z], dtype=float
            )
            for item in msg.poses
            if item.name in self.robot_names
        }
        if len(positions) != N_AGENTS or msg.header.frame_id.lstrip("/") != "world":
            self.latency_counts["incomplete"] += 1
            return
        self.accept_positions(positions, self.stamp_seconds(msg.header.stamp))

    def accept_positions(self, positions, source_time):
        assembly_start = time.perf_counter()
        received_time = self.now_seconds()
        if source_time <= 0.0:
            source_time = received_time
        ordered_positions = np.asarray([positions[name] for name in self.robot_names], dtype=float)
        velocity_start = time.perf_counter()
        with self.state_lock:
            if self.previous_source_time is not None and source_time <= self.previous_source_time:
                return
            if self.previous_positions is None:
                velocities = np.zeros_like(ordered_positions)
            else:
                dt = source_time - self.previous_source_time
                velocities = (ordered_positions - self.previous_positions) / dt
            self.previous_positions = ordered_positions.copy()
            self.previous_source_time = source_time

            rows = np.zeros((N_AGENTS, STATE_WIDTH), dtype=float)
            rows[:, 0:3] = ordered_positions
            rows[:, 3:6] = velocities
            rows[:, 6:8] = self.goals_xy
            rows[:, 8] = self.rate_hz
            self.sequence += 1
            snapshot = StateSnapshot(self.sequence, source_time, received_time, rows)
        self.record_latency("velocity", time.perf_counter() - velocity_start)
        self.record_latency("state_assembly", time.perf_counter() - assembly_start)
        self.queue_snapshot(snapshot)

    def queue_snapshot(self, snapshot):
        run_snapshot = None
        with self.lock:
            if self.pending_snapshot is not None:
                self.latency_counts["replaced"] += 1
            self.pending_snapshot = snapshot
            now = time.perf_counter()
            if not self.inference_running and now - self.last_cycle_start >= self.min_period:
                run_snapshot = self._claim_pending_locked(now)
        if run_snapshot is not None:
            self.process_snapshot(run_snapshot)

    def _claim_pending_locked(self, now):
        snapshot = self.pending_snapshot
        self.pending_snapshot = None
        self.inference_running = True
        self.last_cycle_start = now
        return snapshot

    def process_snapshot(self, snapshot):
        cycle_start = time.perf_counter()
        try:
            state_age = self.now_seconds() - snapshot.source_time
            self.record_latency("state_age_at_start", state_age)
            self.record_latency("callback_and_rate_wait", self.now_seconds() - snapshot.received_time)
            if state_age > float(self.get_parameter("pose_timeout").value):
                self.latency_counts["stale"] += 1
                return

            self.publish_array(self.state_pub, snapshot.state, "state_width", STATE_WIDTH)
            if self.phase == "waiting":
                self.start_takeoff(snapshot)

            if self.phase == "takeoff":
                self.run_takeoff(snapshot)
            elif self.phase == "policy":
                self.run_policy(snapshot)
            elif self.phase == "hold":
                self.run_hold(snapshot)
            elif self.phase == "landing":
                self.run_landing(snapshot)
            elif self.phase == "hold_fault":
                zeros = np.zeros((N_AGENTS, 3), dtype=float)
                self.publish_reference(self.hold_positions, zeros, zeros, snapshot)
            elif self.phase == "landed":
                self.run_landed(snapshot)

            self.latency_counts["processed"] += 1
        finally:
            cycle_time = time.perf_counter() - cycle_start
            self.record_latency("cycle", cycle_time)
            if cycle_time > self.min_period:
                self.latency_counts["deadline_miss"] += 1
            self.finish_cycle()
            self.maybe_report_latency(snapshot.sequence)

    def finish_cycle(self):
        with self.lock:
            self.inference_running = False

    def start_takeoff(self, snapshot):
        if self.mode == "real" and not self.arm_requested:
            if not self.arm_client.service_is_ready():
                return
            request = Arm.Request()
            request.arm = True
            self.arm_client.call_async(request)
            self.arm_requested = True
        self.takeoff_start_positions = snapshot.state[:, 0:3].copy()
        self.phase_start_time = self.now_seconds()
        self.phase = "takeoff"
        self.get_logger().info("Starting low-level cmd_full_state takeoff stream.")

    @staticmethod
    def smooth_step(tau):
        tau = float(np.clip(tau, 0.0, 1.0))
        value = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
        first = 30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4
        second = 60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3
        return value, first, second

    def vertical_reference(self, start_positions, target_z, duration, elapsed):
        duration = max(float(duration), 1e-6)
        value, first, second = self.smooth_step(elapsed / duration)
        delta = target_z - start_positions[:, 2]
        positions = start_positions.copy()
        positions[:, 2] = start_positions[:, 2] + delta * value
        velocities = np.zeros((N_AGENTS, 3), dtype=float)
        velocities[:, 2] = delta * first / duration
        accelerations = np.zeros((N_AGENTS, 3), dtype=float)
        accelerations[:, 2] = delta * second / duration**2
        return positions, velocities, accelerations

    def run_takeoff(self, snapshot):
        now = self.now_seconds()
        duration = float(self.get_parameter("takeoff_duration").value)
        refs = self.vertical_reference(
            self.takeoff_start_positions,
            float(self.get_parameter("hover_height").value),
            duration,
            now - self.phase_start_time,
        )
        self.publish_reference(*refs, snapshot)
        if now - self.phase_start_time < duration:
            return
        hover_height = float(self.get_parameter("hover_height").value)
        hover_epsilon = float(self.get_parameter("hover_epsilon").value)
        if np.all(np.abs(snapshot.state[:, 2] - hover_height) <= hover_epsilon):
            self.phase = "policy"
            self.policy_start_time = now
            self.get_logger().info("Hover confirmed; starting GCBF policy.")
        elif now - self.phase_start_time > float(self.get_parameter("takeoff_timeout").value):
            self.get_logger().error("Timed out waiting for low-level takeoff confirmation.")
            self.hold_positions = snapshot.state[:, 0:3].copy()
            self.phase = "hold_fault"

    def run_policy(self, snapshot):
        if self.now_seconds() - self.policy_start_time > float(self.get_parameter("max_runtime").value):
            self.get_logger().info("GCBF actor reached max runtime; holding position.")
            self.hold_positions = snapshot.state[:, 0:3].copy()
            self.phase = "hold_fault"
            return

        state = snapshot.state
        positions_xy = state[:, 0:2]
        velocities_xy = state[:, 3:5]
        goals_xy = state[:, 6:8]

        section_start = time.perf_counter()
        positions_jax = jnp.asarray(positions_xy, dtype=jnp.float32)
        velocities_jax = jnp.asarray(velocities_xy, dtype=jnp.float32)
        jax.block_until_ready((positions_jax, velocities_jax))
        self.record_latency("input_conversion", time.perf_counter() - section_start)

        section_start = time.perf_counter()
        action_jax, cbf_jax = self.infer_fn(
            positions_jax, velocities_jax, self.goals_jax, self.obstacle_state
        )
        jax.block_until_ready((action_jax, cbf_jax))
        action = np.asarray(action_jax, dtype=float)
        cbf = np.asarray(cbf_jax, dtype=float).reshape(N_AGENTS)
        self.record_latency("compiled_inference", time.perf_counter() - section_start)

        action_age = self.now_seconds() - snapshot.source_time
        self.record_latency("action_age", action_age)
        if action_age > float(self.get_parameter("max_action_age").value):
            self.latency_counts["stale"] += 1
            return

        section_start = time.perf_counter()
        min_distance = pairwise_min_distance(positions_xy)
        if min_distance < self.collision_distance:
            self.get_logger().error(
                f"Pairwise distance {min_distance:.3f} below collision boundary "
                f"{self.collision_distance:.3f}; holding position."
            )
            self.hold_positions = state[:, 0:3].copy()
            self.phase = "hold_fault"
            return

        self.publish_array(self.action_pub, action, "action_width", 2)
        self.publish_array(self.cbf_pub, cbf.reshape(N_AGENTS, 1), "cbf_width", 1)
        goal_errors = np.linalg.norm(goals_xy - positions_xy, axis=1)
        if np.all(goal_errors < float(self.get_parameter("goal_tolerance").value)):
            self.get_logger().info("All Crazyflies reached their goals; starting hold.")
            self.hold_positions = state[:, 0:3].copy()
            self.phase_start_time = self.now_seconds()
            self.phase = "hold"
            return

        accel_xy = action / self.mass
        lookahead_dt = float(self.get_parameter("lookahead_dt").value)
        velocities = np.zeros((N_AGENTS, 3), dtype=float)
        velocities[:, 0:2] = velocities_xy + accel_xy * lookahead_dt
        positions = state[:, 0:3].copy()
        positions[:, 0:2] = (
            positions_xy + velocities_xy * lookahead_dt + 0.5 * accel_xy * lookahead_dt**2
        )
        positions[:, 2] = float(self.get_parameter("hover_height").value)
        accelerations = np.zeros((N_AGENTS, 3), dtype=float)
        accelerations[:, 0:2] = accel_xy
        self.record_latency("safety_lookahead", time.perf_counter() - section_start)
        self.publish_reference(positions, velocities, accelerations, snapshot)

    def run_hold(self, snapshot):
        zeros = np.zeros((N_AGENTS, 3), dtype=float)
        self.publish_reference(self.hold_positions, zeros, zeros, snapshot)
        if self.now_seconds() - self.phase_start_time >= GOAL_HOLD_SECONDS:
            self.landing_start_positions = self.hold_positions.copy()
            self.phase_start_time = self.now_seconds()
            self.phase = "landing"
            self.get_logger().info("Starting low-level cmd_full_state landing stream.")

    def run_landing(self, snapshot):
        now = self.now_seconds()
        refs = self.vertical_reference(
            self.landing_start_positions,
            LAND_HEIGHT,
            LAND_DURATION,
            now - self.phase_start_time,
        )
        self.publish_reference(*refs, snapshot)
        if now - self.phase_start_time < LAND_DURATION:
            return
        measured_z = snapshot.state[:, 2]
        measured_vz = snapshot.state[:, 5]
        tolerance = float(self.get_parameter("hover_epsilon").value)
        if not (np.all(np.abs(measured_z - LAND_HEIGHT) <= tolerance) and np.all(np.abs(measured_vz) <= tolerance)):
            return
        self.hold_positions = refs[0]
        self.phase = "landed"
        self.get_logger().info("Landing confirmed.")
        self.run_landed(snapshot)

    def run_landed(self, snapshot):
        if self.mode == "real" and not self.disarm_requested and self.arm_client.service_is_ready():
            request = Arm.Request()
            request.arm = False
            self.arm_client.call_async(request)
            self.disarm_requested = True
            self.get_logger().info("Landing confirmed; requested hardware disarm.")
        elif self.mode == "sim":
            zeros = np.zeros((N_AGENTS, 3), dtype=float)
            self.publish_reference(self.hold_positions, zeros, zeros, snapshot)

    def publish_reference(self, positions, velocities, accelerations, snapshot):
        section_start = time.perf_counter()
        stamp = rclpy.time.Time(seconds=snapshot.source_time).to_msg()
        for name, position, velocity, acceleration in zip(
            self.robot_names, positions, velocities, accelerations
        ):
            msg = FullState()
            msg.header.stamp = stamp
            msg.header.frame_id = "/world"
            msg.pose.position.x = float(position[0])
            msg.pose.position.y = float(position[1])
            msg.pose.position.z = float(position[2])
            msg.pose.orientation.w = 1.0
            msg.twist.linear.x = float(velocity[0])
            msg.twist.linear.y = float(velocity[1])
            msg.twist.linear.z = float(velocity[2])
            msg.acc.x = float(acceleration[0])
            msg.acc.y = float(acceleration[1])
            msg.acc.z = float(acceleration[2])
            self.command_publishers[name].publish(msg)
        self.record_latency("command_publish", time.perf_counter() - section_start)
        self.record_latency("state_to_command", self.now_seconds() - snapshot.source_time)

    def publish_array(self, publisher, values, width_label, width):
        msg = Float64MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label="agents", size=N_AGENTS, stride=N_AGENTS * width),
            MultiArrayDimension(label=width_label, size=width, stride=width),
        ]
        msg.data = np.asarray(values, dtype=float).reshape(-1).tolist()
        publisher.publish(msg)

    def record_latency(self, name, seconds):
        if self.print_latency:
            self.latency_samples.setdefault(name, []).append(float(seconds) * 1e3)

    def maybe_report_latency(self, sequence):
        if not self.print_latency or time.perf_counter() - self.last_latency_report < 1.0:
            return
        parts = []
        for name, values in sorted(self.latency_samples.items()):
            if values:
                parts.append(
                    f"{name}=latest:{values[-1]:.2f}/mean:{np.mean(values):.2f}/max:{np.max(values):.2f}ms"
                )
        counts = ",".join(f"{key}:{value}" for key, value in self.latency_counts.items())
        self.get_logger().info(f"latency seq={sequence} {' '.join(parts)} counts={counts}")
        self.latency_samples.clear()
        self.last_latency_report = time.perf_counter()


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
