#!/usr/bin/env python3
"""
Joycon Publisher Node
publish joycon data on topic /joycon_command
"""

import rclpy
from rclpy.node import Node
from custom_msgs.msg import JoyconCommand
from std_msgs.msg import Header
from joyconrobotics import JoyconRobotics

import math


class JoyconPublisher(Node):

    def __init__(self):
        super().__init__('joycon_publisher')
        
        self.joycon = JoyconRobotics(device="right",
            offset_euler_rad=[math.pi, 0, 0],
            euler_reverse=[-1, -1, -1],
            direction_reverse=[1, 1, 1])
        self.get_logger().info("Successfully connected to right joycon")
        
        self.publisher_ = self.create_publisher(
            JoyconCommand,
            'joycon_command',
            10
        )
        
        timer_period = 0.001  # 1ms = 1000Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Joycon publisher node started, publishing frequency: {1.0/timer_period:.0f} Hz')
    
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
                msg.x_cartesian = [float(x), float(y), float(z), 
                                   float(roll), float(pitch), float(yaw)]
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
