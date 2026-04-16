#!/usr/bin/env python3
"""
Joycon Publisher Node
"""

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from custom_msgs.msg import JoyconCommand, JoyconDataButton
from std_msgs.msg import Header, String
from joyconrobotics import JoyconRobotics

from .joycon_button_ids import (
    BUTTON_A,
    BUTTON_B,
    BUTTON_PLUS,
    BUTTON_X,
    BUTTON_Y,
)

import math


class JoyconPublisher(Node):
    PHASE_TOPIC = 'controller_phase'
    PHASE_SWITCHING_TO_RESET = 'SWITCHING_TO_RESET'
    PHASE_IN_RESET = 'IN_RESET'


    def __init__(self):
        super().__init__('joycon_publisher')
        
        self.joycon = JoyconRobotics(device="right",
            offset_euler_rad=[math.pi, 0, 0],
            euler_reverse=[-1, -1, -1],
            direction_reverse=[1, 1, 1],
            all_button_return=True)
        # keep startup calibration, but disable recalibration on "+" during runtime.
        self.joycon.reset_joycon = lambda: None
        self.get_logger().info("Successfully connected to right joycon")
        
        self.joycon_cmd_pub = self.create_publisher(
            JoyconCommand,
            'joycon_command',
            10
        )
        self.joycon_btn_pub = self.create_publisher(
            JoyconDataButton,
            'joycon_data_buttons',
            10,
        )
        self._btn_prev = {'x': 0, 'y': 0, 'a': 0, 'b': 0, 'plus': 0}
        
        self.phase_sub = self.create_subscription(
            String,
            self.PHASE_TOPIC,
            self.phase_callback,
            10,
        )
        self.position_bias = [0.0, 0.0, 0.0]
        self.last_reset_request = False
        self.reset_requested = False            # refer to comments in phase_callback function
        self.pending_bias_reset = False         # refer to comments in phase_callback function
        self.hold_translation = False           # refer to comments in phase_callback function
        self.last_xyz_cmd = [0.0, 0.0, 0.0]

        self._cb_group_cmd = MutuallyExclusiveCallbackGroup()
        self._cb_group_btn = MutuallyExclusiveCallbackGroup()

        timer_period = 0.001  # 1ms = 1000Hz
        self.timer_cmd = self.create_timer(
            timer_period,
            self.timer_cmd_callback,
            callback_group=self._cb_group_cmd,
        )
        self.timer_btn = self.create_timer(
            timer_period,
            self.timer_btn_callback,
            callback_group=self._cb_group_btn,
        )

        self.get_logger().info(
            f'Joycon publisher: /joycon_command and /joycon_data_buttons at {1.0 / timer_period:.0f} Hz '
            f'(two timers in separate callback groups)'
        )

    def _publish_button_edges(self, header: Header) -> None:
        """Rising-edge JoyconDataButton for X,Y,A,B,+ (raw HID state)."""
        jc = self.joycon.joycon
        edges = [
            ('x', jc.get_button_x(), BUTTON_X),
            ('y', jc.get_button_y(), BUTTON_Y),
            ('a', jc.get_button_a(), BUTTON_A),
            ('b', jc.get_button_b(), BUTTON_B),
            ('plus', jc.get_button_plus(), BUTTON_PLUS),
        ]
        for key, raw, bid in edges:
            v = 1 if raw else 0
            if v == 1 and self._btn_prev[key] == 0:
                m = JoyconDataButton()
                m.header = header
                m.button_id = bid
                m.pressed = True
                self.joycon_btn_pub.publish(m)
            self._btn_prev[key] = v

    def phase_callback(self, msg):
        phase = msg.data
        if phase == self.PHASE_SWITCHING_TO_RESET:
            # hold_translation: freeze xyz output at last command when switch_controller_node receives
            # reset robot request but is still switching to reset_controller.
            # flag released when switch_controller_node switches to IN_RESET phase.
            self.hold_translation = True
        elif phase == self.PHASE_IN_RESET:
            # only reset translation baseline after controller switched to reset mode.
            if self.reset_requested:
                # pending_bias_reset: one-shot trigger when reset_requested turns True;;
                # apply new transalation command bias on next publish tick.
                self.pending_bias_reset = True
                # reset_requested: latched "+" reset edge; 
                # consume it once  JoyCon "+" button is pressed.
                self.reset_requested = False
            self.hold_translation = False
        else:
            self.hold_translation = False

    def timer_cmd_callback(self):
        try:
            pose, gripper, control_button = self.joycon.get_control()

            if len(pose) >= 6:
                x, y, z, roll, pitch, yaw = pose[:6]

                msg = JoyconCommand()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'joycon_frame'
                # 8 is mapped by joycon-sdk to right JoyCon "+" button event.
                msg.reset_request = (control_button == 8)

                if msg.reset_request and not self.last_reset_request:
                    # request reset now; translation bias reset waits for IN_RESET phase handshake.
                    self.reset_requested = True

                if self.pending_bias_reset:
                    # phase received from switch_controller_node is IN_RESET, reset translation bias
                    self.position_bias = [float(x), float(y), float(z)]
                    self.pending_bias_reset = False

                x_cmd = float(x) - self.position_bias[0]
                y_cmd = float(y) - self.position_bias[1]
                z_cmd = float(z) - self.position_bias[2]
                if self.hold_translation:
                    x_cmd, y_cmd, z_cmd = self.last_xyz_cmd
                else:
                    self.last_xyz_cmd = [x_cmd, y_cmd, z_cmd]

                msg.x_cartesian = [x_cmd, y_cmd, z_cmd,
                                   float(roll), float(pitch), float(yaw)]
                msg.gripper_state = bool(gripper)

                self.last_reset_request = msg.reset_request

                self.joycon_cmd_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Error in timer_cmd_callback: {str(e)}')

    def timer_btn_callback(self):
        """Publish ``JoyconDataButton`` edge events from raw HID (same rate as command timer)."""
        try:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'joycon_frame'
            self._publish_button_edges(header)
        except Exception as e:
            self.get_logger().error(f'Error in timer_btn_callback: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    joycon_publisher = None
    executor = None
    try:
        joycon_publisher = JoyconPublisher()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(joycon_publisher)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error running node: {e}")
    finally:
        if executor is not None:
            executor.shutdown()
        if joycon_publisher is not None:
            joycon_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
