from __future__ import annotations

"""Recorder UI state machine driven by JoyconDataButton edges (see joycon_button_ids)."""

from enum import Enum, auto


from joycon_wrapper.joycon_button_ids import (
    BUTTON_A,
    BUTTON_B,
    BUTTON_PLUS,
    BUTTON_X,
    BUTTON_Y,
)


class RecorderPhase(Enum):
    IDLE = auto()
    RECORDING = auto()
    """
    2 possible phases after stop recording,
    until discard(Y/+) or label success(A) or failure(B).
    """
    WAIT_AFTER_PLUS = auto()
    WAIT_AFTER_Y = auto()
    # After A/B label from Y-stop: wait for + (arm reset) before next IDLE.
    WAIT_AFTER_Y_AB = auto()


class RecorderAction(Enum):
    NOOP = auto()
    START_RECORDING = auto()
    """
    end recording.
    if stopped via +, robot reset already issued on /joycon_command and switch_controller_node;
    if stopped via Y, robot reset is issued by another pressed button event.
    """
    STOP_RECORDING_VIA_PLUS = auto()
    STOP_RECORDING_VIA_Y = auto()
    DISCARD_SESSION = auto()
    COMMIT_SUCCESS = auto()
    COMMIT_FAILURE = auto()
    # + in WAIT_AFTER_Y_AB -> IDLE (arm reset acknowledged).
    ARM_RESET_ACK = auto()


class SessionController:
    """
    interprets button_id edges from joycon_data_buttons
    ("+" button also on topic /joycon_command for robot position reset).

    Rules:
    - IDLE: X starts recording.
    - RECORDING: + or Y stops (+ => wait Y/A/B; Y => wait +/A/B). X ignored.
    - WAIT_AFTER_PLUS: Y discard; A success; B failure -> IDLE.
    - WAIT_AFTER_Y: + discard; A success; B failure -> WAIT_AFTER_Y_AB (not IDLE).
    - WAIT_AFTER_Y_AB: + -> IDLE (arm reset / ready for next session); other keys ignored.
    """

    def __init__(self) -> None:
        self._phase = RecorderPhase.IDLE

    @property
    def phase(self) -> RecorderPhase:
        return self._phase

    @property
    def is_recording(self) -> bool:
        return self._phase == RecorderPhase.RECORDING

    def handle_button_edge(self, button_id: int) -> RecorderAction:
        """Handle one pressed-edge (caller filters pressed==False)."""
        if self._phase == RecorderPhase.IDLE:
            if button_id == BUTTON_X:
                self._phase = RecorderPhase.RECORDING
                return RecorderAction.START_RECORDING
            return RecorderAction.NOOP

        if self._phase == RecorderPhase.RECORDING:
            if button_id == BUTTON_PLUS:
                self._phase = RecorderPhase.WAIT_AFTER_PLUS
                return RecorderAction.STOP_RECORDING_VIA_PLUS
            if button_id == BUTTON_Y:
                self._phase = RecorderPhase.WAIT_AFTER_Y
                return RecorderAction.STOP_RECORDING_VIA_Y
            return RecorderAction.NOOP

        if self._phase == RecorderPhase.WAIT_AFTER_PLUS:
            if button_id == BUTTON_Y:
                self._phase = RecorderPhase.IDLE
                return RecorderAction.DISCARD_SESSION
            if button_id == BUTTON_A:
                self._phase = RecorderPhase.IDLE
                return RecorderAction.COMMIT_SUCCESS
            if button_id == BUTTON_B:
                self._phase = RecorderPhase.IDLE
                return RecorderAction.COMMIT_FAILURE
            return RecorderAction.NOOP

        if self._phase == RecorderPhase.WAIT_AFTER_Y:
            if button_id == BUTTON_PLUS:
                self._phase = RecorderPhase.IDLE
                return RecorderAction.DISCARD_SESSION
            if button_id == BUTTON_A:
                self._phase = RecorderPhase.WAIT_AFTER_Y_AB
                return RecorderAction.COMMIT_SUCCESS
            if button_id == BUTTON_B:
                self._phase = RecorderPhase.WAIT_AFTER_Y_AB
                return RecorderAction.COMMIT_FAILURE
            return RecorderAction.NOOP

        if self._phase == RecorderPhase.WAIT_AFTER_Y_AB:
            if button_id == BUTTON_PLUS:
                self._phase = RecorderPhase.IDLE
                return RecorderAction.ARM_RESET_ACK
            return RecorderAction.NOOP

        return RecorderAction.NOOP
