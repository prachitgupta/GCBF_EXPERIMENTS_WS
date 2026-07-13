import argparse
import math
from pathlib import Path
from types import SimpleNamespace

import jax
import jax.numpy as jnp
import jax.random as jr
import numpy as np

from gcbfplus.algo import make_algo
from gcbfplus.env import make_env


N_AGENTS = 8
DEFAULT_RATE_HZ = 50.0
DEFAULT_LOOKAHEAD_DT = 0.05
DEFAULT_HOVER_HEIGHT = 0.5
DEFAULT_GOAL_TOLERANCE = 0.12
DEFAULT_MAX_RUNTIME = 30.0
TAKEOFF_DURATION = 3.0
LAND_DURATION = 3.0
LAND_HEIGHT = 0.03
POSE_TIMEOUT = 10.0
POSE_STALE_SECONDS = 1.0


def package_root() -> Path:
    return Path(__file__).resolve().parents[1]


def workspace_root() -> Path:
    return package_root().parents[1]


def default_model_dir() -> Path:
    return package_root() / "pretrained_diffuser" / "DoubleIntegrator" / "gcbfdiffuser"


def default_crazyflies_yaml() -> Path:
    return workspace_root() / "src" / "crazyflie_vicon_bringup" / "config" / "crazyflies_8.yaml"


def parse_scalar(value):
    value = value.strip()
    if value in {"null", "None"}:
        return None
    if value in {"true", "false"}:
        return value == "true"
    try:
        if "." in value or "e" in value.lower():
            return float(value)
        return int(value)
    except ValueError:
        return value


def load_config(path: Path) -> SimpleNamespace:
    try:
        import yaml

        with path.open("r") as f:
            return yaml.load(f, Loader=yaml.UnsafeLoader)
    except ImportError:
        values = {}
        for line in path.read_text().splitlines():
            stripped = line.strip()
            if not stripped or stripped.startswith("#") or ":" not in stripped:
                continue
            key, value = stripped.split(":", 1)
            if key.startswith("!!"):
                continue
            values[key.strip()] = parse_scalar(value)
        return SimpleNamespace(**values)


def load_crazyflies(path: Path):
    try:
        import yaml

        with path.open("r") as f:
            data = yaml.safe_load(f)
        robots = data["robots"]
        items = [
            (name, value["initial_position"])
            for name, value in robots.items()
            if value.get("enabled", False)
        ]
    except ImportError:
        items = []
        in_robots = False
        current_name = None
        current_enabled = False
        current_position = None
        for line in path.read_text().splitlines():
            if line.startswith("robots:"):
                in_robots = True
                continue
            if in_robots and line and not line.startswith(" "):
                break
            if not in_robots:
                continue
            if line.startswith("  ") and not line.startswith("    ") and line.strip().endswith(":"):
                if current_name and current_enabled and current_position is not None:
                    items.append((current_name, current_position))
                current_name = line.strip()[:-1]
                current_enabled = False
                current_position = None
            elif current_name and "enabled:" in line:
                current_enabled = "true" in line.lower()
            elif current_name and "initial_position:" in line:
                raw = line.split("[", 1)[1].split("]", 1)[0]
                current_position = [float(part.strip()) for part in raw.split(",")]
        if current_name and current_enabled and current_position is not None:
            items.append((current_name, current_position))

    if len(items) != N_AGENTS:
        raise RuntimeError(f"Expected {N_AGENTS} enabled Crazyflies in {path}, found {len(items)}.")

    def angle(item):
        pos = item[1]
        return math.atan2(pos[1], pos[0]) % (2.0 * math.pi)

    return sorted(items, key=angle)


def make_algo_from_checkpoint(model_dir: Path, step: int, area_size: float, max_runtime: float):
    config = load_config(model_dir / "config.yaml")
    env = make_env(
        env_id="DoubleIntegrator",
        num_agents=N_AGENTS,
        num_obs=0,
        area_size=area_size,
        max_step=max(1, int(max_runtime / 0.03)),
        max_travel=None,
    )
    algo = make_algo(
        algo=getattr(config, "algo", "gcbf+"),
        env=env,
        node_dim=env.node_dim,
        edge_dim=env.edge_dim,
        state_dim=env.state_dim,
        action_dim=env.action_dim,
        n_agents=env.num_agents,
        gnn_layers=config.gnn_layers,
        batch_size=config.batch_size,
        buffer_size=config.buffer_size,
        horizon=config.horizon,
        lr_actor=config.lr_actor,
        lr_cbf=config.lr_cbf,
        alpha=config.alpha,
        eps=getattr(config, "eps", 0.02),
        inner_epoch=getattr(config, "inner_epoch", 8),
        loss_action_coef=config.loss_action_coef,
        loss_unsafe_coef=config.loss_unsafe_coef,
        loss_safe_coef=config.loss_safe_coef,
        loss_h_dot_coef=config.loss_h_dot_coef,
        max_grad_norm=getattr(config, "max_grad_norm", 2.0),
        seed=config.seed,
    )
    algo.load(str(model_dir / "models"), step)
    return env, algo


def make_graph(env, obstacle_state, positions_xy, velocities_xy, goals_xy):
    agent_states = jnp.concatenate(
        [jnp.asarray(positions_xy, dtype=jnp.float32), jnp.asarray(velocities_xy, dtype=jnp.float32)],
        axis=1,
    )
    goal_states = jnp.concatenate(
        [jnp.asarray(goals_xy, dtype=jnp.float32), jnp.zeros((N_AGENTS, 2), dtype=jnp.float32)],
        axis=1,
    )
    return env.get_graph(env.EnvState(agent_states, goal_states, obstacle_state))


def pairwise_min_distance(positions_xy):
    min_distance = np.inf
    for i in range(len(positions_xy)):
        for j in range(i + 1, len(positions_xy)):
            min_distance = min(min_distance, np.linalg.norm(positions_xy[i] - positions_xy[j]))
    return float(min_distance)


class PoseSource:
    def __init__(self, node):
        from tf2_msgs.msg import TFMessage

        self._positions = {}
        self._subscription = node.create_subscription(TFMessage, "/tf", self._tf_callback, 10)

    def _tf_callback(self, msg):
        for transform in msg.transforms:
            child = transform.child_frame_id.lstrip("/")
            parent = transform.header.frame_id.lstrip("/")
            if parent != "world":
                continue
            position = transform.transform.translation
            stamp = transform.header.stamp
            self._positions[child] = (
                np.array([position.x, position.y, position.z], dtype=float),
                float(stamp.sec) + float(stamp.nanosec) * 1e-9,
            )

    def position(self, cf):
        if cf.poseStamped:
            return np.array(cf.get_position(), dtype=float)
        entry = self._positions.get(cf.prefix.lstrip("/"))
        if entry is None:
            return None
        return entry[0]

    def stamp(self, cf):
        if cf.poseStamped:
            stamp = cf.poseStamped
            return float(stamp["timestamp_sec"]) + float(stamp["timestamp_nsec"]) * 1e-9
        entry = self._positions.get(cf.prefix.lstrip("/"))
        if entry is None:
            return None
        return entry[1]


def wait_for_poses(cfs, pose_source, time_helper, timeout):
    start = time_helper.time()
    while time_helper.time() - start < timeout:
        if all(pose_source.position(cf) is not None for cf in cfs):
            return
        time_helper.sleepForRate(20.0)
    missing = [cf.prefix for cf in cfs if pose_source.position(cf) is None]
    raise RuntimeError(f"Missing pose updates for: {', '.join(missing)}")


def poses_are_stale(cfs, pose_source, now):
    for cf in cfs:
        if not cf.poseStamped:
            continue
        msg_time = pose_source.stamp(cf)
        if msg_time is None:
            return True
        if msg_time > 0.0 and now - msg_time > POSE_STALE_SECONDS:
            return True
    return False


def current_positions(cfs, pose_source):
    return np.array([pose_source.position(cf) for cf in cfs], dtype=float)


def notify_setpoints_stop(cfs):
    for cf in cfs:
        cf.notifySetpointsStop()


def land_all(cfs, time_helper):
    for cf in cfs:
        cf.land(targetHeight=LAND_HEIGHT, duration=LAND_DURATION)
    time_helper.sleep(LAND_DURATION + 1.0)


def save_results(path: Path, results):
    if results["positions"]:
        serializable = {
            "positions": np.stack(results["positions"], axis=0),
            "velocities": np.stack(results["velocities"], axis=0),
            "actions": np.stack(results["actions"], axis=0),
            "goals": results["goals"],
            "min_distance": np.asarray(results["min_distance"], dtype=float),
            "cbf": np.stack(results["cbf"], axis=0) if results["cbf"] else np.empty((0, N_AGENTS)),
        }
        np.savez(path, **serializable)


def run(args):
    from crazyflie_py import Crazyswarm
    import rclpy

    model_dir = Path(args.model_dir).expanduser().resolve()
    crazyflies_yaml = Path(args.crazyflies_yaml).expanduser().resolve()
    output = Path(args.output).expanduser().resolve()

    env, algo = make_algo_from_checkpoint(model_dir, args.step, args.area_size, args.max_runtime)
    act_fn = jax.jit(algo.act)
    cbf_fn = jax.jit(algo.get_cbf)

    robot_items = load_crazyflies(crazyflies_yaml)
    robot_names = [name for name, _ in robot_items]
    starts_xy = np.array([position[:2] for _, position in robot_items], dtype=float)
    goals_xy = np.roll(starts_xy, shift=N_AGENTS // 2, axis=0)

    initial_graph = env.reset(jr.PRNGKey(args.seed))
    obstacle_state = initial_graph.env_states.obstacle
    mass = float(env._params["m"])
    collision_distance = 2.0 * float(env._params["car_radius"])

    swarm = Crazyswarm()
    time_helper = swarm.timeHelper
    cf_by_name = swarm.allcfs.crazyfliesByName
    cfs = [cf_by_name[name] for name in robot_names]
    pose_source = PoseSource(swarm.allcfs)

    results = {
        "positions": [],
        "velocities": [],
        "actions": [],
        "goals": goals_xy,
        "min_distance": [],
        "cbf": [],
    }

    try:
        for cf in cfs:
            cf.takeoff(targetHeight=args.hover_height, duration=TAKEOFF_DURATION)
        time_helper.sleep(TAKEOFF_DURATION + 1.0)
        wait_for_poses(cfs, pose_source, time_helper, args.pose_timeout)

        last_positions = current_positions(cfs, pose_source)
        last_time = time_helper.time()
        velocities_xy = np.zeros((N_AGENTS, 2), dtype=float)
        start_time = last_time

        while time_helper.time() - start_time < args.max_runtime:
            now = time_helper.time()
            positions = current_positions(cfs, pose_source)
            positions_xy = positions[:, :2]
            sample_dt = max(now - last_time, 1.0 / args.rate_hz)
            velocities_xy = (positions_xy - last_positions[:, :2]) / sample_dt

            if poses_are_stale(cfs, pose_source, now):
                raise RuntimeError("Pose data became stale.")

            graph = make_graph(env, obstacle_state, positions_xy, velocities_xy, goals_xy)
            action = np.asarray(env.clip_action(act_fn(graph)), dtype=float)
            if action.shape != (N_AGENTS, 2) or not np.all(np.isfinite(action)):
                raise RuntimeError(f"Invalid actor output shape or values: {action.shape}")

            accel_xy = action / mass
            velocity_refs_xy = velocities_xy + accel_xy * args.lookahead_dt
            position_refs_xy = (
                positions_xy
                + velocities_xy * args.lookahead_dt
                + 0.5 * accel_xy * args.lookahead_dt**2
            )

            min_distance = pairwise_min_distance(positions_xy)
            if min_distance < collision_distance:
                raise RuntimeError(
                    f"Pairwise distance {min_distance:.3f} below collision boundary {collision_distance:.3f}."
                )

            h = np.asarray(cbf_fn(graph)).reshape(N_AGENTS)
            results["positions"].append(positions.copy())
            results["velocities"].append(velocities_xy.copy())
            results["actions"].append(action.copy())
            results["min_distance"].append(min_distance)
            results["cbf"].append(h)

            goal_errors = np.linalg.norm(goals_xy - positions_xy, axis=1)
            if np.all(goal_errors < args.goal_tolerance):
                break

            for cf, pos_xy, vel_xy, acc_xy in zip(cfs, position_refs_xy, velocity_refs_xy, accel_xy):
                pos_ref = np.array([pos_xy[0], pos_xy[1], args.hover_height])
                vel_ref = np.array([vel_xy[0], vel_xy[1], 0.0])
                acc_ref = np.array([acc_xy[0], acc_xy[1], 0.0])
                cf.cmdFullState(pos_ref, vel_ref, acc_ref, 0.0, np.zeros(3))

            last_positions = positions
            last_time = now
            time_helper.sleepForRate(args.rate_hz)
    finally:
        save_results(output, results)
        notify_setpoints_stop(cfs)
        time_helper.sleep(0.2)
        land_all(cfs, time_helper)
        if rclpy.ok():
            rclpy.shutdown()

    print(f"Saved results: {output}")


def parse_args():
    parser = argparse.ArgumentParser(description="Run trained DoubleIntegrator GCBF actor on Crazyswarm2 sim.")
    parser.add_argument("--model-dir", default=str(default_model_dir()))
    parser.add_argument("--crazyflies-yaml", default=str(default_crazyflies_yaml()))
    parser.add_argument("--step", type=int, default=1000)
    parser.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ)
    parser.add_argument("--lookahead-dt", type=float, default=DEFAULT_LOOKAHEAD_DT)
    parser.add_argument("--hover-height", type=float, default=DEFAULT_HOVER_HEIGHT)
    parser.add_argument("--goal-tolerance", type=float, default=DEFAULT_GOAL_TOLERANCE)
    parser.add_argument("--max-runtime", type=float, default=DEFAULT_MAX_RUNTIME)
    parser.add_argument("--pose-timeout", type=float, default=POSE_TIMEOUT)
    parser.add_argument("--area-size", type=float, default=4.0)
    parser.add_argument("--seed", type=int, default=2)
    parser.add_argument("--output", default="gcbf_crazyswarm_double_integrator.npz")
    return parser.parse_args()


def main():
    run(parse_args())


if __name__ == "__main__":
    main()
