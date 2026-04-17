# ruff: noqa

"""Replay recorded joint-space trajectory from trajectory.h5 (franky JointWaypointMotion).

ROS 2 CLI: ``ros2 run data_recorder replay_trajectory --ros-args -p trajectory_h5:=/path/to/trajectory.h5``

Recommended: ``ros2 launch data_recorder replay_trajectory.launch.py`` after
``conda activate franka_data`` (uses that env's Python; see launch file).

Depends on: franky, h5py, numpy, rclpy (install franky in the conda env you use)."""

from __future__ import annotations

import dataclasses
import sys
from typing import Optional

import franky
import h5py
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


@dataclasses.dataclass
class Args:
    trajectory_h5: str
    robot_ip: str = "172.16.0.2"
    dynamic_factor: float = 0.2
    realtime_config: str = "ignore"  # ignore | enforce

    joint_dataset: str = "observation/robot_state/joint_positions"
    timestamp_dataset: str = "observation/timestamp/robot_state"

    # Use every k-th sample (1 = all samples).
    stride: int = 1
    # If > 0, first perform a separate move to the first joint pose over this many milliseconds.
    go_to_start_ms: int = 0
    # If True, skip the first recorded pose in the main motion (use after --go-to-start-ms reached q[0]).
    skip_first_after_go_to_start: bool = True


class ReplayTrajectoryNode(Node):
    def __init__(self) -> None:
        super().__init__("replay_trajectory")

        self.declare_parameter("trajectory_h5", "")
        self.declare_parameter("robot_ip", "172.16.0.2")
        self.declare_parameter("dynamic_factor", 0.2)
        self.declare_parameter("realtime_config", "ignore")
        self.declare_parameter("joint_dataset", "observation/robot_state/joint_positions")
        self.declare_parameter("timestamp_dataset", "observation/timestamp/robot_state")
        self.declare_parameter("stride", 1)
        self.declare_parameter("go_to_start_ms", 0)
        self.declare_parameter("skip_first_after_go_to_start", True)

    def build_args(self, trajectory_override: str = "") -> Optional[Args]:
        traj = trajectory_override.strip() or self.get_parameter("trajectory_h5").get_parameter_value().string_value.strip()
        if not traj:
            self.get_logger().error(
                "trajectory_h5 is empty. Set e.g. "
                "'--ros-args -p trajectory_h5:=/path/to/trajectory.h5' "
                "or pass the .h5 path after '--'."
            )
            return None

        return Args(
            trajectory_h5=traj,
            robot_ip=self.get_parameter("robot_ip").get_parameter_value().string_value,
            dynamic_factor=self.get_parameter("dynamic_factor").get_parameter_value().double_value,
            realtime_config=self.get_parameter("realtime_config").get_parameter_value().string_value,
            joint_dataset=self.get_parameter("joint_dataset").get_parameter_value().string_value,
            timestamp_dataset=self.get_parameter("timestamp_dataset").get_parameter_value().string_value,
            stride=int(self.get_parameter("stride").get_parameter_value().integer_value),
            go_to_start_ms=int(self.get_parameter("go_to_start_ms").get_parameter_value().integer_value),
            skip_first_after_go_to_start=self.get_parameter(
                "skip_first_after_go_to_start"
            ).get_parameter_value().bool_value,
        )


def _realtime_config(name: str) -> franky.RealtimeConfig:
    n = name.lower().replace("-", "_")
    mapping = {
        "ignore": franky.RealtimeConfig.Ignore,
        "enforce": franky.RealtimeConfig.Enforce,
    }
    if n not in mapping:
        raise ValueError(f"realtime_config must be one of {list(mapping)}, got {name!r}")
    return mapping[n]


def _resolve_joint_dataset(f: h5py.File, preferred: str) -> str:
    if preferred in f:
        return preferred
    candidates = [
        "observation/robot_state/joint_position",
        "observation/robot_state/joint_positions",
        "joint_position",
        "joint_positions",
    ]
    for c in candidates:
        if c in f:
            print(f"Using joint dataset {c!r} (preferred {preferred!r} missing).", file=sys.stderr)
            return c
    raise KeyError(f"No joint position dataset found. Tried {preferred!r} and {candidates}")


def load_joint_trajectory(
    path: str,
    joint_dataset: str,
    timestamp_dataset: str,
    stride: int,
) -> tuple[np.ndarray, np.ndarray]:
    with h5py.File(path, "r") as f:
        jkey = _resolve_joint_dataset(f, joint_dataset)
        q = np.asarray(f[jkey][...], dtype=np.float64)
        if q.ndim != 2 or q.shape[1] != 7:
            raise ValueError(f"Expected joint shape (T, 7), got {q.shape} from {jkey!r}")

        if timestamp_dataset not in f:
            raise KeyError(f"Missing timestamp dataset {timestamp_dataset!r}")
        t = np.asarray(f[timestamp_dataset][...], dtype=np.int64)
        if t.shape[0] != q.shape[0]:
            raise ValueError(f"Length mismatch: joints {q.shape[0]} vs timestamps {t.shape[0]}")

    if stride < 1:
        raise ValueError("stride must be >= 1")
    q = q[::stride]
    t = t[::stride]
    if q.shape[0] < 2:
        raise ValueError("Need at least 2 samples after stride to define segment timings.")
    return q, t


def _dt_ms_from_timestamps(t_ns: np.ndarray) -> np.ndarray:
    """Durations in ms between consecutive samples; length T-1, all >= 1."""
    dt_ns = np.diff(t_ns.astype(np.int64))
    dt_ms = np.maximum(1, np.rint(dt_ns / 1.0e6).astype(np.int64))
    return dt_ms


def run_replay(args: Args) -> None:
    q, t = load_joint_trajectory(
        args.trajectory_h5,
        args.joint_dataset,
        args.timestamp_dataset,
        args.stride,
    )
    dt_ms = _dt_ms_from_timestamps(t)

    rc = _realtime_config(args.realtime_config)
    robot = franky.Robot(args.robot_ip, realtime_config=rc)
    robot.relative_dynamics_factor = franky.RelativeDynamicsFactor(
        velocity=args.dynamic_factor,
        acceleration=args.dynamic_factor,
        jerk=args.dynamic_factor,
    )
    robot.recover_from_errors()

    q0 = q[0].tolist()

    if args.go_to_start_ms > 0:
        start_motion = franky.JointWaypointMotion(
            [franky.JointWaypoint(q0, minimum_time=franky.Duration(int(args.go_to_start_ms)))]
        )
        print(f"Moving to first recorded pose over {args.go_to_start_ms} ms...")
        robot.move(start_motion, asynchronous=False)

    start_idx = 1 if args.go_to_start_ms > 0 and args.skip_first_after_go_to_start else 0
    if start_idx >= q.shape[0]:
        raise ValueError("No waypoints left after start_idx; reduce stride or disable skip_first_after_go_to_start.")

    waypoints: list[franky.JointWaypoint] = []
    for i in range(start_idx, q.shape[0]):
        # Segment q[i-1] -> q[i] lasts ~(t[i]-t[i-1]) = dt_ms[i-1]. For i==0 (robot -> q[0]), use dt_ms[0].
        seg_ms = int(dt_ms[i - 1] if i > 0 else dt_ms[0])
        waypoints.append(franky.JointWaypoint(q[i].tolist(), minimum_time=franky.Duration(seg_ms)))

    motion = franky.JointWaypointMotion(waypoints)
    print(
        f"Replaying {len(waypoints)} segments (from sample {start_idx} to {q.shape[0] - 1}), "
        f"robot {args.robot_ip!r}..."
    )
    robot.move(motion, asynchronous=False)
    print("Done.")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ReplayTrajectoryNode()
    try:
        argv = remove_ros_args(sys.argv)
        trajectory_override = argv[1] if len(argv) > 1 else ""
        replay_args = node.build_args(trajectory_override=trajectory_override)
        if replay_args is None:
            return
        run_replay(replay_args)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
