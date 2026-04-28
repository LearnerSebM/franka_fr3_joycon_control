from __future__ import annotations

import json
import os
from datetime import datetime
from typing import Any, Dict, List, Mapping, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from custom_msgs.msg import JoyconDataButton

from data_recorder.camera_stream import (
    CAMERA_TYPE_EXTERIOR,
    CAMERA_TYPE_WRIST,
    CameraStream,
    build_camera_stream,
)
from data_recorder.hdf5_writer import write_recording
from data_recorder.session_controller import RecorderAction, RecorderPhase, SessionController
from data_recorder.topic_stream import JointStateSnapshot, TopicStream
from joycon_wrapper.joycon_button_ids import BUTTON_X, BUTTON_Y


DEFAULT_WRIST_SERIAL = "352122273343"
DEFAULT_EXTERIOR_SERIAL = "311322303242"
DEFAULT_PLACEHOLDER_SERIAL = "zero_exterior_2"
DEFAULT_LANGUAGE_INSTRUCTION = "default prompt"
DEFAULT_ANNOTATIONS_FILENAME = "aggregated-annotations-latest.json"


def _joint_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class DataRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("data_recorder")

        self.declare_parameter("namespace", "NS_1")
        self.declare_parameter("sample_hz", 15.0)
        self.declare_parameter("ring_capacity", 1000)
        self.declare_parameter("output_directory", os.path.expanduser("~/joycon_recordings"))
        self.declare_parameter("file_prefix", "recording")
        self.declare_parameter("language_instruction", DEFAULT_LANGUAGE_INSTRUCTION)
        self.declare_parameter("annotations_filename", DEFAULT_ANNOTATIONS_FILENAME)
        self.declare_parameter("enable_cameras", True)
        self.declare_parameter("wrist_camera_serial", DEFAULT_WRIST_SERIAL)
        self.declare_parameter("exterior_camera_serial", DEFAULT_EXTERIOR_SERIAL)
        self.declare_parameter("placeholder_camera_serial", DEFAULT_PLACEHOLDER_SERIAL)
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 30)
        self.declare_parameter("camera_frame_timeout_s", 2.0)
        self.declare_parameter("camera_working_dir_base", "")
        self.declare_parameter("camera_queue_maxsize", 64)

        ns = self.get_parameter("namespace").get_parameter_value().string_value.strip().strip("/")
        sample_hz = self.get_parameter("sample_hz").get_parameter_value().double_value
        ring_cap = self.get_parameter("ring_capacity").get_parameter_value().integer_value
        self._output_dir = os.path.expanduser(
            self.get_parameter("output_directory").get_parameter_value().string_value
        )
        self._file_prefix = self.get_parameter("file_prefix").get_parameter_value().string_value
        self._language_instruction = (
            self.get_parameter("language_instruction").get_parameter_value().string_value.strip()
            or DEFAULT_LANGUAGE_INSTRUCTION
        )
        self._annotations_filename = (
            self.get_parameter("annotations_filename").get_parameter_value().string_value.strip()
            or DEFAULT_ANNOTATIONS_FILENAME
        )
        self._enable_cameras = bool(
            self.get_parameter("enable_cameras").get_parameter_value().bool_value
        )
        self._wrist_serial = (
            self.get_parameter("wrist_camera_serial").get_parameter_value().string_value.strip()
            or DEFAULT_WRIST_SERIAL
        )
        self._exterior_serial = (
            self.get_parameter("exterior_camera_serial").get_parameter_value().string_value.strip()
            or DEFAULT_EXTERIOR_SERIAL
        )
        self._placeholder_serial = (
            self.get_parameter("placeholder_camera_serial").get_parameter_value().string_value.strip()
            or DEFAULT_PLACEHOLDER_SERIAL
        )
        self._cam_width = int(
            self.get_parameter("camera_width").get_parameter_value().integer_value
        )
        self._cam_height = int(
            self.get_parameter("camera_height").get_parameter_value().integer_value
        )
        self._cam_fps = int(
            self.get_parameter("camera_fps").get_parameter_value().integer_value
        )
        self._cam_timeout = float(
            self.get_parameter("camera_frame_timeout_s").get_parameter_value().double_value
        )
        _cam_dir_base = (
            self.get_parameter("camera_working_dir_base")
            .get_parameter_value()
            .string_value.strip()
        )
        self._cam_dir: Optional[str] = (
            os.path.expanduser(_cam_dir_base) if _cam_dir_base else None
        )
        self._cam_queue_size = max(
            2,
            int(self.get_parameter("camera_queue_maxsize").get_parameter_value().integer_value),
        )

        self._joint_topic = f"/{ns}/joint_states"
        self._button_topic = f"/{ns}/joycon_data_buttons"

        self._topic_stream = TopicStream(capacity=max(1, int(ring_cap)))
        self._session = SessionController()
        self._cameras: CameraStream | None = None

        writer_fps = max(1, int(round(sample_hz))) if sample_hz > 0 else 15
        if self._enable_cameras:
            camera_fps = max(1, int(self._cam_fps))
            try:
                self._cameras = build_camera_stream(
                    wrist_serial=self._wrist_serial,
                    exterior_serial=self._exterior_serial,
                    placeholder_serial=self._placeholder_serial,
                    width=self._cam_width,
                    height=self._cam_height,
                    camera_fps=camera_fps,
                    writer_fps=writer_fps,
                    frame_timeout_s=self._cam_timeout,
                    working_dir_base=self._cam_dir,
                    queue_maxsize=self._cam_queue_size,
                )
            except Exception as exc:
                self._cameras = None
                self.get_logger().error(
                    f"Failed to initialize cameras (continuing without cameras): {exc}"
                )

        self.create_subscription(
            JointState,
            self._joint_topic,
            self._on_joint_state,
            _joint_qos(),
        )
        self.create_subscription(
            JoyconDataButton,
            self._button_topic,
            self._on_joycon_button,
            10,
        )

        period = 1.0 / sample_hz if sample_hz > 0.0 else 0.1
        self.create_timer(period, self._on_sample_timer)

        self.create_service(Trigger, "~/start_recording", self._handle_start)
        self.create_service(Trigger, "~/stop_recording", self._handle_stop)

        self.get_logger().info(
            f"data_recorder: joint_states={self._joint_topic}, "
            f"joycon_data_buttons={self._button_topic}, sample_hz={sample_hz}, "
            f"ring_capacity={ring_cap}\n"
            f"language_instruction='{self._language_instruction}'\n"
            f"cameras_enabled={self._cameras is not None} "
            f"(wrist={self._wrist_serial}, exterior={self._exterior_serial}, "
            f"placeholder={self._placeholder_serial}; "
            f"camera_fps={self._cam_fps}, writer_fps={writer_fps}; "
            f"camera_working_dir_base={self._cam_dir!r})\n"
            f"Press X to start recording."
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._topic_stream.set_latest(msg)

    def _on_joycon_button(self, msg: JoyconDataButton) -> None:
        if not msg.pressed:
            return
        action = self._session.handle_button_edge(int(msg.button_id))
        self._switch_recorder_action(action)

    def _on_sample_timer(self) -> None:
        if not self._session.is_recording:
            return
        if self._cameras is not None:
            if not self._cameras.readers_ready(timeout_s=0.0):
                return
        t_ros_ns = self._topic_stream.try_sample_tick()
        if t_ros_ns is None:
            return
        if self._cameras is not None:
            ok = self._cameras.on_sample_tick(t_ros_ns)
            if not ok:
                # Extremely unlikely (ready but frame went away); log once.
                self.get_logger().warning(
                    "Camera tick skipped after joint sample committed; "
                    "HDF5 T and MP4 frame count may diverge by one."
                )

    def _make_episode_paths(self, outcome: str) -> Tuple[str, str, str]:
        # DROID-like episode layout: <output>/<outcome>/<day>/<episode>/trajectory.h5
        now = datetime.now()
        day = now.strftime("%Y-%m-%d")
        episode_id = now.strftime("%Y%m%d%H%M%S")
        episode_folder = f"{self._file_prefix}_{now.strftime('%a_%b_%d_%H-%M-%S_%Y')}"
        episode_dir = os.path.join(self._output_dir, outcome, day, episode_folder)
        os.makedirs(episode_dir, exist_ok=True)
        trajectory_path = os.path.join(episode_dir, "trajectory.h5")
        return episode_dir, trajectory_path, episode_id

    def _handle_start(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        action = self._session.handle_button_edge(BUTTON_X)
        if action == RecorderAction.START_RECORDING:
            self._switch_recorder_action(action)
            resp.success = True
            resp.message = "recording"
        else:
            resp.success = False
            resp.message = f"cannot start (phase={self._session.phase.name})"
        return resp

    def _handle_stop(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        action = self._session.handle_button_edge(BUTTON_Y)
        if action == RecorderAction.STOP_RECORDING_VIA_Y:
            self._switch_recorder_action(action)
            resp.success = True
            resp.message = "stopped_pending_label"
        else:
            resp.success = False
            resp.message = f"not recording (phase={self._session.phase.name})"
        return resp

    def _switch_recorder_action(self, action: RecorderAction) -> None:
        if action == RecorderAction.NOOP:
            return
        if action == RecorderAction.START_RECORDING:
            self._topic_stream.begin_recording()
            if self._cameras is not None:
                try:
                    self._cameras.begin_recording()
                except Exception as exc:
                    self.get_logger().error(
                        f"CameraStream.begin_recording failed: {exc}"
                    )
            self.get_logger().info(
                f"\n--------------------------------\n"
                f"Recording started (X):\n"
                f"Press Y to stop recording;\n"
                f"Press + to stop recording and reset robot position."
            )
            return
        if action in (
            RecorderAction.STOP_RECORDING_VIA_PLUS,
            RecorderAction.STOP_RECORDING_VIA_Y,
        ):
            self._topic_stream.end_recording()
            if self._cameras is not None:
                try:
                    self._cameras.end_recording()
                except Exception as exc:
                    self.get_logger().error(
                        f"CameraStream.end_recording failed: {exc}"
                    )
            discard_hint = (
                "Press Y to discard trajectory;\n"
                if action == RecorderAction.STOP_RECORDING_VIA_PLUS
                else "Press + to discard trajectory;\n"
            )
            self.get_logger().info(
                f"\n--------------------------------\n"
                f"Recording stopped ({'+' if action == RecorderAction.STOP_RECORDING_VIA_PLUS else 'Y'})\n"
                f"{discard_hint}"
                f"Press A to commit trajectory as success;\n"
                f"Press B to commit trajectory as failure."
            )
            return
        if action == RecorderAction.DISCARD_SESSION:
            self._topic_stream.clear_storage()
            if self._cameras is not None:
                try:
                    self._cameras.clear_storage()
                except Exception as exc:
                    self.get_logger().error(
                        f"CameraStream.clear_storage failed: {exc}"
                    )
            self.get_logger().info(
                f"\n--------------------------------\n"
                f"Trajectory discarded (no HDF5)\n"
                f"Press X to start next recording."
            )
            return
        if action == RecorderAction.COMMIT_SUCCESS:
            follow = (
                "Press + to reset arm, then press X to start next recording."
                if self._session.phase == RecorderPhase.WAIT_AFTER_Y_AB
                else "Press X to start next recording."
            )
            self.get_logger().info(
                f"\n--------------------------------\n"
                f"Trajectory committed as success.\n"
                f"{follow}"
            )
            self._commit_trajectory("success")
            return
        if action == RecorderAction.COMMIT_FAILURE:
            follow = (
                "Press + to reset arm, then press X to start next recording."
                if self._session.phase == RecorderPhase.WAIT_AFTER_Y_AB
                else "Press X to start next recording."
            )
            self.get_logger().info(
                f"\n--------------------------------\n"
                f"Trajectory committed as failure.\n"
                f"{follow}"
            )
            self._commit_trajectory("failure")
            return
        if action == RecorderAction.ARM_RESET_ACK:
            self.get_logger().info(
                f"\n--------------------------------\n"
                f"Press X to start recording."
            )
            return

    def _commit_trajectory(self, outcome: str) -> None:
        episode_dir, trajectory_path, episode_id = self._make_episode_paths(outcome)

        snap = self._topic_stream.pack_snapshot()
        if snap is None:
            snap = JointStateSnapshot(
                [],
                np.zeros(0, dtype=np.int64),
                np.zeros((0, 0), dtype=np.float64),
                np.zeros((0, 0), dtype=np.float64),
                np.zeros((0, 0), dtype=np.float64),
                0,
                0,
            )
            self.get_logger().warning("Commit: no queued samples; writing placeholder HDF5")

        camera_info: Dict[str, Dict[str, Any]] = {}
        frame_count = 0
        if self._cameras is not None:
            try:
                camera_info, frame_count = self._cameras.finalize_to(episode_dir)
            except Exception as exc:
                self.get_logger().error(
                    f"CameraStream.finalize_to failed: {exc}; writing trajectory without MP4s"
                )
                camera_info = {}

        if (
            self._cameras is not None
            and camera_info
            and snap.valid_count != frame_count
        ):
            self.get_logger().warning(
                f"Mismatch between joint samples ({snap.valid_count}) "
                f"and camera frames ({frame_count}); MP4 T may not align with HDF5."
            )

        hdf5_metadata = {
            "language_instruction": self._language_instruction,
            "episode_id": episode_id,
            "source": "franka_fr3_joycon_control",
            "outcome": outcome,
        }

        try:
            write_recording(
                trajectory_path,
                snap,
                trajectory_outcome=outcome,
                camera_info=camera_info,
                metadata=hdf5_metadata,
            )
        except Exception as exc:
            self.get_logger().error(f"write_recording failed: {exc}")
            return

        self._write_episode_metadata(
            episode_dir=episode_dir,
            episode_id=episode_id,
            outcome=outcome,
            camera_info=camera_info,
        )
        self._update_aggregated_annotations(episode_id=episode_id)

        self._topic_stream.clear_storage()
        self.get_logger().info(
            f"Wrote HDF5 outcome={outcome}: {trajectory_path} "
            f"(samples={snap.valid_count}, frames={frame_count})"
        )

    def _write_episode_metadata(
        self,
        *,
        episode_dir: str,
        episode_id: str,
        outcome: str,
        camera_info: Mapping[str, Mapping[str, Any]],
    ) -> None:
        wrist, ext1, ext2 = self._classify_cameras(camera_info)
        record = {
            "episode_id": episode_id,
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "outcome": outcome,
            "language_instruction": self._language_instruction,
            "wrist_cam_serial": wrist,
            "ext1_cam_serial": ext1,
            "ext2_cam_serial": ext2,
        }
        metadata_path = os.path.join(episode_dir, f"metadata_{episode_id}.json")
        try:
            with open(metadata_path, "w", encoding="utf-8") as f:
                json.dump(record, f, ensure_ascii=False, indent=2)
        except OSError as exc:
            self.get_logger().error(f"failed to write {metadata_path}: {exc}")

    def _update_aggregated_annotations(self, *, episode_id: str) -> None:
        annotations_path = os.path.join(self._output_dir, self._annotations_filename)
        os.makedirs(os.path.dirname(os.path.abspath(annotations_path)) or ".", exist_ok=True)
        payload: Dict[str, Dict[str, str]] = {}
        if os.path.exists(annotations_path):
            try:
                with open(annotations_path, "r", encoding="utf-8") as f:
                    loaded = json.load(f)
                if isinstance(loaded, dict):
                    payload = {str(k): dict(v) for k, v in loaded.items() if isinstance(v, dict)}
            except (OSError, json.JSONDecodeError) as exc:
                self.get_logger().warning(
                    f"annotations file unreadable, will overwrite: {exc}"
                )
        payload[episode_id] = {
            "language_instruction1": self._language_instruction,
            "language_instruction2": self._language_instruction,
            "language_instruction3": self._language_instruction,
        }
        try:
            with open(annotations_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=False, indent=2)
        except OSError as exc:
            self.get_logger().error(f"failed to write {annotations_path}: {exc}")

    def _classify_cameras(
        self, camera_info: Mapping[str, Mapping[str, Any]]
    ) -> Tuple[Optional[str], Optional[str], Optional[str]]:
        wrist_serials: List[str] = []
        exterior_serials: List[str] = []
        for serial, info in camera_info.items():
            cam_type = int(info.get("camera_type", CAMERA_TYPE_EXTERIOR))
            if cam_type == CAMERA_TYPE_WRIST:
                wrist_serials.append(serial)
            else:
                exterior_serials.append(serial)
        wrist = wrist_serials[0] if wrist_serials else None
        ext1 = exterior_serials[0] if exterior_serials else None
        ext2 = exterior_serials[1] if len(exterior_serials) > 1 else None
        return wrist, ext1, ext2

    def destroy_node(self) -> bool:
        if self._session.is_recording:
            self._topic_stream.end_recording()
            if self._cameras is not None:
                try:
                    self._cameras.end_recording()
                except Exception:
                    pass
        self._topic_stream.clear_storage()
        if self._cameras is not None:
            try:
                self._cameras.shutdown()
            except Exception:
                pass
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = DataRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
