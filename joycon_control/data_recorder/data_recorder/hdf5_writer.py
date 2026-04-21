from __future__ import annotations

"""Write one DROID-style trajectory.h5 from a joint-state snapshot."""

import os
from typing import TYPE_CHECKING, Any, Dict, Mapping, Optional, Sequence

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


def _unpack_snapshot(snapshot: "JointStateSnapshot", n: int
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
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


def _normalize_camera_info(
    camera_info: Optional[Mapping[str, Mapping[str, Any]]],
    n: int,
    fallback_timestamps: np.ndarray,
) -> Dict[str, Dict[str, np.ndarray]]:
    """
    Validate and normalize per-camera metadata to the shape expected on disk.
    """
    if not camera_info:
        return {}
    normalized: Dict[str, Dict[str, np.ndarray]] = {}
    for serial, info in camera_info.items():
        cam_type = int(info.get("camera_type", 1))
        timestamps = np.asarray(info.get("timestamps_ns", []), dtype=np.int64)
        if timestamps.shape != (n,):
            # Pad or truncate to stay aligned with the joint-state timeline.
            aligned = np.zeros((n,), dtype=np.int64)
            if fallback_timestamps.shape == (n,):
                aligned[:] = fallback_timestamps
            copy_len = min(n, timestamps.shape[0])
            if copy_len > 0:
                aligned[:copy_len] = timestamps[:copy_len]
            timestamps = aligned
        type_array = np.full((n,), cam_type, dtype=np.int32)
        normalized[str(serial)] = {
            "camera_type": type_array,
            "timestamps_ns": timestamps,
        }
    return normalized


def _make_droid_payload(
    snapshot: "JointStateSnapshot",
    n: int,
    camera_info: Optional[Mapping[str, Mapping[str, Any]]],
) -> Mapping[str, Any]:
    arm_pos, arm_vel, gripper_pos = _unpack_snapshot(snapshot, n)
    t_ros = np.asarray(snapshot.t_ros_ns[:n], dtype=np.int64)
    movement_enabled = np.ones((n,), dtype=np.bool_)

    cameras = _normalize_camera_info(camera_info, n, t_ros)
    camera_type_group = {serial: data["camera_type"] for serial, data in cameras.items()}
    camera_timestamps = {
        f"{serial}_frame_received": data["timestamps_ns"]
        for serial, data in cameras.items()
    }

    return {
        "observation": {
            "robot_state": {
                "joint_positions": arm_pos,
                "gripper_position": gripper_pos,
            },
            "controller_info": {
                "movement_enabled": movement_enabled,
            },
            "camera_type": camera_type_group,
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
    camera_info: Optional[Mapping[str, Mapping[str, Any]]] = None,
    camera_serials: Optional[Sequence[str]] = None,
    metadata: Optional[Mapping[str, Any]] = None,
) -> None:
    """
    Persist snapshot to HDF5. If t_record_ns is given, shape must match valid_count
    (wall time when each sample was taken in the recorder).

    ``camera_info`` supersedes ``camera_serials``. When neither is provided, the
    trajectory is written without any camera entries -- could be used for tests without
    the camera pipeline.
    """
    os.makedirs(os.path.dirname(os.path.abspath(file_path)) or ".", exist_ok=True)
    n = max(0, int(snapshot.valid_count))

    effective_camera_info: Optional[Mapping[str, Mapping[str, Any]]]
    if camera_info:
        effective_camera_info = camera_info
    elif camera_serials:
        fallback_timestamps = np.asarray(snapshot.t_ros_ns[:n], dtype=np.int64)
        effective_camera_info = {
            serial: {"camera_type": 1, "timestamps_ns": fallback_timestamps}
            for serial in camera_serials
        }
    else:
        effective_camera_info = None

    payload = _make_droid_payload(snapshot, n, effective_camera_info)

    with h5py.File(file_path, "w") as h5:
        h5.attrs["schema_version"] = SCHEMA_VERSION
        if trajectory_outcome is not None:
            h5.attrs["trajectory_outcome"] = trajectory_outcome
        if metadata:
            for key, value in metadata.items():
                if value is None:
                    continue
                h5.attrs[key] = value
        if t_record_ns is not None:
            h5.attrs["t_record_ns_start"] = (
                int(np.asarray(t_record_ns[:1], dtype=np.int64)[0]) if n > 0 else -1
            )
        _write_nested(h5, payload)
