from __future__ import annotations

import os
from typing import TYPE_CHECKING, Any, Mapping, Optional, Sequence

import numpy as np

try:
    import h5py
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "data_recorder requires h5py. Install with: pip install h5py"
    ) from exc

if TYPE_CHECKING:
    from data_recorder.topic_stream import JointStateSnapshot


SCHEMA_VERSION = 1


def _as_float64_array(values: np.ndarray, n: int) -> np.ndarray:
    arr = np.asarray(values[:n], dtype=np.float64)
    if arr.ndim != 2:
        return np.zeros((n, 0), dtype=np.float64)
    return arr


def _ensure_width(arr: np.ndarray, width: int) -> np.ndarray:
    if arr.shape[1] >= width:
        return arr[:, :width]
    pad = np.zeros((arr.shape[0], width - arr.shape[1]), dtype=arr.dtype)
    return np.concatenate((arr, pad), axis=1)


def _select_arm_and_gripper(snapshot: "JointStateSnapshot", n: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    joint_names = [name.lower() for name in snapshot.joint_names]
    pos = _as_float64_array(snapshot.position, n)
    vel = _as_float64_array(snapshot.velocity, n)

    if pos.shape[1] == 0:
        return np.zeros((n, 7), dtype=np.float64), np.zeros((n, 7), dtype=np.float64), np.zeros((n,), dtype=np.float64)

    gripper_indices = [i for i, name in enumerate(joint_names) if ("gripper" in name or "finger" in name)]
    arm_indices = [i for i in range(pos.shape[1]) if i not in gripper_indices]

    if len(arm_indices) < 7:
        arm_indices = list(range(min(7, pos.shape[1])))
    arm_indices = arm_indices[:7]

    arm_pos = _ensure_width(pos[:, arm_indices], 7)
    arm_vel = _ensure_width(vel[:, arm_indices], 7)

    if gripper_indices:
        gripper_position = np.mean(pos[:, gripper_indices], axis=1)
    elif pos.shape[1] > 7:
        gripper_position = pos[:, 7]
    else:
        gripper_position = pos[:, -1]

    return arm_pos, arm_vel, np.asarray(gripper_position, dtype=np.float64)


def _make_droid_payload(
    snapshot: "JointStateSnapshot",
    n: int,
    camera_serials: Sequence[str],
) -> Mapping[str, Any]:
    arm_pos, arm_vel, gripper_pos = _select_arm_and_gripper(snapshot, n)
    t_ros = np.asarray(snapshot.t_ros_ns[:n], dtype=np.int64)
    movement_enabled = np.ones((n,), dtype=np.bool_)

    camera_type = {serial: np.zeros((n,), dtype=np.int32) for serial in camera_serials}
    camera_timestamps = {f"{serial}_frame_received": t_ros for serial in camera_serials}

    return {
        "observation": {
            "robot_state": {
                "joint_positions": arm_pos,
                "gripper_position": gripper_pos,
            },
            "controller_info": {
                "movement_enabled": movement_enabled,
            },
            "camera_type": camera_type,
            "timestamp": {
                "robot_state": t_ros,
                "cameras": camera_timestamps,
            },
        },
        "action": {
            "joint_velocity": arm_vel,
            "gripper_position": gripper_pos,
        },
    }


def _write_nested(group: "h5py.Group", payload: Mapping[str, Any]) -> None:
    for key, value in payload.items():
        if isinstance(value, Mapping):
            child = group.create_group(key)
            _write_nested(child, value)
            continue
        group.create_dataset(key, data=np.asarray(value))


def write_recording(
    file_path: str,
    snapshot: "JointStateSnapshot",
    t_record_ns: Optional[np.ndarray] = None,
    trajectory_outcome: Optional[str] = None,
    camera_serials: Optional[Sequence[str]] = None,
    metadata: Optional[Mapping[str, Any]] = None,
) -> None:
    """
    Persist snapshot to HDF5. If t_record_ns is given, shape must match valid_count
    (wall time when each sample was taken in the recorder).
    """
    os.makedirs(os.path.dirname(os.path.abspath(file_path)) or ".", exist_ok=True)
    n = max(0, int(snapshot.valid_count))
    serials = tuple(camera_serials or ("camera_placeholder",))
    payload = _make_droid_payload(snapshot, n, serials)

    with h5py.File(file_path, "w") as h5:
        h5.attrs["schema_version"] = SCHEMA_VERSION
        if trajectory_outcome is not None:
            h5.attrs["trajectory_outcome"] = trajectory_outcome
        if metadata:
            for key, value in metadata.items():
                h5.attrs[key] = value
        if t_record_ns is not None:
            h5.attrs["t_record_ns_start"] = int(np.asarray(t_record_ns[:1], dtype=np.int64)[0]) if n > 0 else -1
        _write_nested(h5, payload)
