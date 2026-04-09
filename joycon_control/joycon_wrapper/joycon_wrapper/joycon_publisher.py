#!/usr/bin/env python3
"""
Joycon Publisher Node
publish joycon data on topic /joycon_command
"""

import rclpy
from rclpy.node import Node
from custom_msgs.msg import JoyconCommand
from std_msgs.msg import Header, String
from joyconrobotics import JoyconRobotics

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
        
        self.publisher_ = self.create_publisher(
            JoyconCommand,
            'joycon_command',
            10
        )
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
        
        timer_period = 0.001  # 1ms = 1000Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Joycon publisher node started, publishing frequency: {1.0/timer_period:.0f} Hz')

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

    def timer_callback(self):
        """callback function to read joycon data and publish"""
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
                
                self.publisher_.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f'Error reading joycon data: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        joycon_publisher = JoyconPublisher()
        rclpy.spin(joycon_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error running node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
