from __future__ import annotations

"""RealSense camera pipeline for DROID-style recording.

Manages two live RealSense cameras (wrist D405 + exterior D465) and one
all-zero placeholder stream so the produced trajectory always has the
wrist + ext1 + ext2 triplet required by the openpi/DROID conversion
scripts. Color frames are written as BGR into per-camera MP4 files that
land under ``<episode_dir>/recordings/MP4/<serial>.mp4``.
"""

"""This script is fully vibe-coded without further inspection"""

import os
import queue
import shutil
import tempfile
import threading
from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

try:
    import cv2
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "data_recorder requires opencv-python. Install with: pip install opencv-python"
    ) from exc

try:
    import pyrealsense2 as rs  # type: ignore
except ImportError:  # pragma: no cover
    rs = None  # RealSense readers will fail at construction if this is None.


CAMERA_TYPE_WRIST = 0
CAMERA_TYPE_EXTERIOR = 1


@dataclass
class CameraSpec:
    """Static description of one camera entry."""

    serial: str
    camera_type: int
    is_placeholder: bool = False


class _FrameSource:
    """Common reader interface used by CameraStream."""

    def get_bgr_image(self) -> Optional[np.ndarray]:
        raise NotImplementedError

    def stop(self) -> None:
        return None


class RealSenseReader(_FrameSource):
    """Background-threaded RealSense color stream reader.

    Mirrors the pattern used in ``inference_franka.py``: the RealSense
    pipeline keeps producing frames in a background thread while the main
    thread consumes the latest frame on demand.
    """

    def __init__(
        self,
        *,
        serial: str,
        width: int,
        height: int,
        fps: int,
        frame_timeout_s: float = 2.0,
    ) -> None:
        if rs is None:
            raise RuntimeError(
                "pyrealsense2 is not available; activate the conda env with RealSense SDK installed."
            )
        self._serial = serial
        self._width = int(width)
        self._height = int(height)
        self._fps = int(fps)
        self._frame_timeout_s = float(frame_timeout_s)

        self._pipeline = rs.pipeline()
        self._config = rs.config()
        self._config.enable_device(serial)
        self._config.enable_stream(
            rs.stream.color, self._width, self._height, rs.format.bgr8, self._fps
        )
        self._pipeline.start(self._config)

        self._latest_bgr: Optional[np.ndarray] = None
        self._frame_lock = threading.Lock()
        self._frame_ready = threading.Event()
        self._stop_event = threading.Event()
        self._reader_thread = threading.Thread(
            target=self._frame_reader_loop, daemon=True
        )
        self._reader_thread.start()

    @property
    def serial(self) -> str:
        return self._serial

    @property
    def resolution_hw(self) -> Tuple[int, int]:
        return (self._height, self._width)

    def _frame_reader_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                frames = self._pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                bgr_image = np.asanyarray(color_frame.get_data())
                with self._frame_lock:
                    self._latest_bgr = bgr_image
                self._frame_ready.set()
            except Exception:
                continue

    def get_bgr_image(self) -> Optional[np.ndarray]:
        if not self._frame_ready.is_set():
            return None
        with self._frame_lock:
            if self._latest_bgr is None:
                return None
            return self._latest_bgr.copy()

    def wait_until_ready(self, timeout_s: Optional[float] = None) -> bool:
        wait = self._frame_timeout_s if timeout_s is None else float(timeout_s)
        return self._frame_ready.wait(timeout=wait)

    def stop(self) -> None:
        self._stop_event.set()
        try:
            self._reader_thread.join(timeout=self._frame_timeout_s)
        except RuntimeError:
            pass
        try:
            self._pipeline.stop()
        except Exception:
            pass


class ZeroPlaceholderReader(_FrameSource):
    """Synthetic reader that always returns an all-zero BGR frame.

    Used as the third exterior camera to satisfy DROID's wrist + 2 exterior
    expectation without a physical device.
    """

    def __init__(self, *, serial: str, width: int, height: int) -> None:
        self._serial = serial
        self._width = int(width)
        self._height = int(height)
        self._zero_frame = np.zeros((self._height, self._width, 3), dtype=np.uint8)

    @property
    def serial(self) -> str:
        return self._serial

    @property
    def resolution_hw(self) -> Tuple[int, int]:
        return (self._height, self._width)

    def get_bgr_image(self) -> Optional[np.ndarray]:
        return self._zero_frame.copy()

    def wait_until_ready(self, timeout_s: Optional[float] = None) -> bool:
        return True


class CameraStream:
    """Record per-tick BGR frames from multiple cameras into per-serial MP4 files.

    The API intentionally mirrors ``TopicStream`` (begin/tick/end/clear/finalize)
    so the ROS2 node can drive both subsystems uniformly from the Joycon
    state machine.

    RealSense devices are opened at ``camera_fps``; MP4 metadata uses
    ``writer_fps`` (typically the joint-state ``sample_hz``) so time bases stay
    aligned with one stored row per recorder tick. When ``camera_fps`` exceeds
    ``writer_fps``, each tick simply grabs the latest frame.
    """

    MP4_FOURCC = "mp4v"
    _SENTINEL = object()

    def __init__(
        self,
        *,
        specs: Sequence[CameraSpec],
        readers: Mapping[str, _FrameSource],
        width: int,
        height: int,
        writer_fps: int,
        capacity: int = 10_000,
        working_dir_base: Optional[str] = None,
        queue_maxsize: int = 64,
    ) -> None:
        if not specs:
            raise ValueError("CameraStream requires at least one CameraSpec")

        seen = set()
        for spec in specs:
            if spec.serial in seen:
                raise ValueError(f"duplicate camera serial: {spec.serial}")
            if spec.serial not in readers:
                raise ValueError(f"reader for serial {spec.serial} is missing")
            seen.add(spec.serial)

        self._specs: List[CameraSpec] = list(specs)
        self._readers: Dict[str, _FrameSource] = dict(readers)
        self._width = int(width)
        self._height = int(height)
        self._writer_fps = max(1, int(writer_fps))
        self._capacity = max(1, int(capacity))
        self._working_dir_base = working_dir_base
        self._queue_maxsize = max(2, int(queue_maxsize))

        self._lock = threading.Lock()
        self._writers: Dict[str, "cv2.VideoWriter"] = {}
        self._timestamps: Dict[str, List[int]] = {}
        self._working_dir: Optional[str] = None
        self._recording = False
        self._frame_count = 0

        self._queue: queue.Queue = queue.Queue(maxsize=self._queue_maxsize)
        self._writer_thread: Optional[threading.Thread] = None

    @property
    def specs(self) -> Sequence[CameraSpec]:
        return tuple(self._specs)

    @property
    def is_recording(self) -> bool:
        with self._lock:
            return self._recording

    def readers_ready(self, timeout_s: float = 2.0) -> bool:
        """Wait until all live readers have produced at least one frame."""
        deadline_per_reader = max(0.0, float(timeout_s))
        for reader in self._readers.values():
            wait_fn = getattr(reader, "wait_until_ready", None)
            if wait_fn is None:
                continue
            if not wait_fn(deadline_per_reader):
                return False
        return True

    def _drain_queue(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break

    def _stop_writer_thread(self) -> None:
        thread = self._writer_thread
        if thread is None or not thread.is_alive():
            self._writer_thread = None
            return
        try:
            self._queue.put(self._SENTINEL, timeout=5.0)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put(self._SENTINEL, timeout=5.0)
            except queue.Full:
                pass
        thread.join(timeout=60.0)
        self._writer_thread = None

    def _writer_loop(self) -> None:
        while True:
            item = self._queue.get()
            if item is self._SENTINEL:
                break
            t_ros_ns, frames = item
            with self._lock:
                if not self._writers:
                    continue
                if self._frame_count >= self._capacity:
                    continue
                for spec in self._specs:
                    image = frames[spec.serial]
                    if (
                        image.shape[0] != self._height
                        or image.shape[1] != self._width
                    ):
                        image = cv2.resize(image, (self._width, self._height))
                    if image.dtype != np.uint8:
                        image = image.astype(np.uint8)
                    self._writers[spec.serial].write(image)
                    self._timestamps[spec.serial].append(int(t_ros_ns))
                self._frame_count += 1

    def begin_recording(self) -> None:
        self._stop_writer_thread()
        self._drain_queue()
        with self._lock:
            self._cleanup_working_dir_locked()
            self._working_dir = tempfile.mkdtemp(
                prefix="joycon_camera_",
                dir=self._working_dir_base,
            )
            fourcc = cv2.VideoWriter_fourcc(*self.MP4_FOURCC)
            self._writers = {}
            self._timestamps = {spec.serial: [] for spec in self._specs}
            for spec in self._specs:
                target_path = os.path.join(self._working_dir, f"{spec.serial}.mp4")
                writer = cv2.VideoWriter(
                    target_path,
                    fourcc,
                    float(self._writer_fps),
                    (self._width, self._height),
                )
                if not writer.isOpened():
                    for w in self._writers.values():
                        w.release()
                    self._writers = {}
                    self._cleanup_working_dir_locked()
                    raise RuntimeError(
                        f"failed to open VideoWriter for {target_path}"
                    )
                self._writers[spec.serial] = writer
            self._recording = True
            self._frame_count = 0

        self._writer_thread = threading.Thread(
            target=self._writer_loop,
            daemon=True,
            name="camera_mp4_writer",
        )
        self._writer_thread.start()

    def on_sample_tick(self, t_ros_ns: int) -> bool:
        """
        Enqueue one frame set for async write. Returns True if queued.

        If any live reader does not yet have a frame ready, the tick is
        skipped wholesale so that every MP4 stays in lockstep with the
        joint-state time axis. If the write queue is full, returns False
        without blocking the ROS executor.
        """
        with self._lock:
            recording = self._recording
            writers_ok = bool(self._writers)
        if not recording or not writers_ok:
            return False

        frames: Dict[str, np.ndarray] = {}
        for spec in self._specs:
            reader = self._readers[spec.serial]
            image = reader.get_bgr_image()
            if image is None:
                return False
            frames[spec.serial] = image

        with self._lock:
            if not self._recording or not self._writers:
                return False

        try:
            self._queue.put_nowait((int(t_ros_ns), frames))
        except queue.Full:
            return False
        return True

    def end_recording(self) -> None:
        with self._lock:
            self._recording = False
        self._stop_writer_thread()
        with self._lock:
            for writer in self._writers.values():
                try:
                    writer.release()
                except Exception:
                    pass
            self._writers = {}

    def clear_storage(self) -> None:
        with self._lock:
            self._recording = False
        self._stop_writer_thread()
        with self._lock:
            for writer in self._writers.values():
                try:
                    writer.release()
                except Exception:
                    pass
            self._writers = {}
            self._timestamps = {}
            self._frame_count = 0
            self._cleanup_working_dir_locked()

    def finalize_to(
        self, episode_dir: str
    ) -> Tuple[Dict[str, Dict[str, object]], int]:
        """
        Move pending MP4s under ``episode_dir/recordings/MP4/`` and return metadata.

        Returns ``(camera_info, frame_count)`` where camera_info maps each
        serial to ``{"camera_type": int, "timestamps_ns": np.ndarray, "mp4_path": str}``.
        Must be called after ``end_recording`` (writers already closed).
        """
        with self._lock:
            if self._recording:
                raise RuntimeError("finalize_to called while still recording")
            mp4_dir = os.path.join(episode_dir, "recordings", "MP4")
            os.makedirs(mp4_dir, exist_ok=True)
            camera_info: Dict[str, Dict[str, object]] = {}
            frame_count = self._frame_count
            for spec in self._specs:
                timestamps = np.asarray(
                    self._timestamps.get(spec.serial, []), dtype=np.int64
                )
                target_path = os.path.join(mp4_dir, f"{spec.serial}.mp4")
                if self._working_dir is not None:
                    source_path = os.path.join(self._working_dir, f"{spec.serial}.mp4")
                    if os.path.exists(source_path):
                        shutil.move(source_path, target_path)
                camera_info[spec.serial] = {
                    "camera_type": int(spec.camera_type),
                    "timestamps_ns": timestamps,
                    "mp4_path": target_path,
                }
            self._cleanup_working_dir_locked()
            self._timestamps = {}
            self._frame_count = 0
            return camera_info, int(frame_count)

    def shutdown(self) -> None:
        self.clear_storage()
        for reader in self._readers.values():
            try:
                reader.stop()
            except Exception:
                continue

    def _cleanup_working_dir_locked(self) -> None:
        if self._working_dir is not None:
            shutil.rmtree(self._working_dir, ignore_errors=True)
            self._working_dir = None


def build_camera_stream(
    *,
    wrist_serial: str,
    exterior_serial: str,
    placeholder_serial: str,
    width: int,
    height: int,
    camera_fps: int,
    writer_fps: int,
    frame_timeout_s: float = 2.0,
    working_dir_base: Optional[str] = None,
    queue_maxsize: int = 64,
) -> CameraStream:
    """Convenience factory for the wrist + exterior + zero-placeholder layout."""
    cam_fps = max(1, int(camera_fps))
    specs: List[CameraSpec] = [
        CameraSpec(serial=wrist_serial, camera_type=CAMERA_TYPE_WRIST),
        CameraSpec(serial=exterior_serial, camera_type=CAMERA_TYPE_EXTERIOR),
        CameraSpec(
            serial=placeholder_serial,
            camera_type=CAMERA_TYPE_EXTERIOR,
            is_placeholder=True,
        ),
    ]
    readers: Dict[str, _FrameSource] = {
        wrist_serial: RealSenseReader(
            serial=wrist_serial,
            width=width,
            height=height,
            fps=cam_fps,
            frame_timeout_s=frame_timeout_s,
        ),
        exterior_serial: RealSenseReader(
            serial=exterior_serial,
            width=width,
            height=height,
            fps=cam_fps,
            frame_timeout_s=frame_timeout_s,
        ),
        placeholder_serial: ZeroPlaceholderReader(
            serial=placeholder_serial, width=width, height=height
        ),
    }
    return CameraStream(
        specs=specs,
        readers=readers,
        width=width,
        height=height,
        writer_fps=writer_fps,
        working_dir_base=working_dir_base,
        queue_maxsize=queue_maxsize,
    )
