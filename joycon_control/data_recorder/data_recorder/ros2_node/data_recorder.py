from __future__ import annotations

import os
from datetime import datetime
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from custom_msgs.msg import JoyconDataButton

from data_recorder.hdf5_writer import write_recording
from data_recorder.session_controller import RecorderAction, RecorderPhase, SessionController
from data_recorder.topic_stream import JointStateSnapshot, TopicStream
from joycon_wrapper.joycon_button_ids import BUTTON_X, BUTTON_Y


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

        ns = self.get_parameter("namespace").get_parameter_value().string_value.strip().strip("/")
        sample_hz = self.get_parameter("sample_hz").get_parameter_value().double_value
        ring_cap = self.get_parameter("ring_capacity").get_parameter_value().integer_value
        self._output_dir = os.path.expanduser(
            self.get_parameter("output_directory").get_parameter_value().string_value
        )
        self._file_prefix = self.get_parameter("file_prefix").get_parameter_value().string_value

        self._joint_topic = f"/{ns}/joint_states"
        self._button_topic = f"/{ns}/joycon_data_buttons"

        self._topic_stream = TopicStream(capacity=max(1, int(ring_cap)))
        self._last_written_path = ""
        self._session = SessionController()

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
        self._topic_stream.on_sample_tick()

    def _make_output_path(self) -> str:
        os.makedirs(self._output_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(self._output_dir, f"{self._file_prefix}_{ts}.h5")

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
            # X => + => Y or X => Y => +
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
            # TODO: work from _commit_trajectory()
            # self._commit_trajectory("success")
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
            # self._commit_trajectory("failure")
            return
        if action == RecorderAction.ARM_RESET_ACK:
            # X => Y => A/B => +
            self.get_logger().info(
                f"\n--------------------------------\n"
                f"Press X to start recording."
            )
            return

    def _commit_trajectory(self, outcome: str) -> None:
        path = self._make_output_path()
        self._last_written_path = path
        snap = self._topic_stream.get_snapshot_for_hdf5()
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
            self.get_logger().warning("Commit: no ring layout; writing placeholder HDF5")
        write_recording(path, snap, trajectory_outcome=outcome)
        self._topic_stream.clear_storage()
        self.get_logger().info(
            f"Wrote HDF5 outcome={outcome}: {path} (samples={snap.valid_count})"
        )

    def destroy_node(self) -> bool:
        if self._session.is_recording:
            self._topic_stream.end_recording()
        self._topic_stream.clear_storage()
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
