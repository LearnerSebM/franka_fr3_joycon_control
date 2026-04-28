from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
from sensor_msgs.msg import JointState


def _stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


@dataclass
class JointStateSnapshot:
    """
    arrays for HDF5 export
    """

    joint_names: List[str]
    t_ros_ns: np.ndarray
    position: np.ndarray
    velocity: np.ndarray
    effort: np.ndarray
    valid_count: int
    head: int


class TopicStream:
    """
    holds latest JointState from subscription; when recording, each timer tick appends
    one sample to a fixed-capacity queue (oldest dropped when full).
    """

    def __init__(self, capacity: int = 1000) -> None:
        self._capacity = max(1, int(capacity))
        self._lock = threading.Lock()
        self._latest: JointState | None = None
        self._recording = False
        self._joint_dim = 0
        self._names: List[str] = []
        self._t_ros: np.ndarray | None = None
        self._pos: np.ndarray | None = None
        self._vel: np.ndarray | None = None
        self._eff: np.ndarray | None = None
        self._count = 0

    @property
    def capacity(self) -> int:
        return self._capacity

    @property
    def is_recording(self) -> bool:
        return self._recording

    def set_latest(self, msg: JointState) -> None:
        with self._lock:
            self._latest = msg

    def get_latest(self) -> JointState | None:
        with self._lock:
            return self._latest

    def begin_recording(self) -> None:
        with self._lock:
            self._recording = True
            self._count = 0
            self._joint_dim = 0
            self._names = []
            self._t_ros = None
            self._pos = None
            self._vel = None
            self._eff = None

    def end_recording(self) -> None:
        with self._lock:
            self._recording = False

    def clear_storage(self) -> None:
        """
        clear queued samples after discard or commit;
        does not change _recording state.
        """
        with self._lock:
            self._count = 0
            self._joint_dim = 0
            self._names = []
            self._t_ros = None
            self._pos = None
            self._vel = None
            self._eff = None

    def try_sample_tick(self) -> int | None:
        """
        Called by data_recorder at sample_hz while recording;
        uses latest JointState if available.
        Returns ``None`` when the stream is not recording or when no
        JointState message has been observed yet, otherwise returns msg timestamp.
        Callers can use the returned timestamp to align other per-tick streams (cameras).
        """
        with self._lock:
            if not self._recording:
                return None
            msg = self._latest
            if msg is None:
                return None
            self._allocate_buffers(len(msg.name))
            assert self._t_ros is not None and self._pos is not None
            self._names = list(msg.name)
            t_ros_ns = _stamp_to_ns(msg.header.stamp)
            pos, vel, eff = self._unpack_msg(msg)
            if self._count < self._capacity:
                idx = self._count
                self._count += 1
            else:
                self._t_ros[:-1] = self._t_ros[1:]
                self._pos[:-1] = self._pos[1:]
                self._vel[:-1] = self._vel[1:]
                self._eff[:-1] = self._eff[1:]
                idx = self._capacity - 1
            self._t_ros[idx] = t_ros_ns
            self._pos[idx] = pos
            self._vel[idx] = vel
            self._eff[idx] = eff
            return int(t_ros_ns)

    def _allocate_buffers(self, n_joint: int) -> None:
        if self._joint_dim == n_joint and self._t_ros is not None:
            # buffer already allocated
            return
        self._joint_dim = n_joint
        self._t_ros = np.zeros(self._capacity, dtype=np.int64)
        self._pos = np.zeros((self._capacity, n_joint), dtype=np.float64)
        self._vel = np.zeros((self._capacity, n_joint), dtype=np.float64)
        self._eff = np.zeros((self._capacity, n_joint), dtype=np.float64)

    def _unpack_msg(self, msg: JointState) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        d = self._joint_dim

        def row(arr: List[float]) -> np.ndarray:
            x = np.asarray(arr, dtype=np.float64)
            out = np.zeros((d,), dtype=np.float64)
            n = min(d, x.size)
            if n > 0:
                out[:n] = x[:n]
            return out

        return row(list(msg.position)), row(list(msg.velocity)), row(list(msg.effort))

    def pack_snapshot(self) -> JointStateSnapshot | None:
        """
        Queued oldest-to-newest order for min(count, capacity) samples.
        Safe to call after end_recording (buffers retained until begin_recording).
        """
        with self._lock:
            if self._t_ros is None or self._pos is None or self._joint_dim == 0:
                return None
            n = min(self._count, self._capacity)
            if n == 0:
                return JointStateSnapshot(
                    joint_names=list(self._names),
                    t_ros_ns=np.zeros(0, dtype=np.int64),
                    position=np.zeros((0, self._joint_dim), dtype=np.float64),
                    velocity=np.zeros((0, self._joint_dim), dtype=np.float64),
                    effort=np.zeros((0, self._joint_dim), dtype=np.float64),
                    valid_count=0,
                    head=0,
                )
            return JointStateSnapshot(
                joint_names=list(self._names),
                t_ros_ns=self._t_ros[:n].copy(),
                position=self._pos[:n].copy(),
                velocity=self._vel[:n].copy(),
                effort=self._eff[:n].copy(),
                valid_count=int(n),
                head=0,
            )
