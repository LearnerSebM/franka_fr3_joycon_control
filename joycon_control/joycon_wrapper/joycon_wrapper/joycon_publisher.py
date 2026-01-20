#!/usr/bin/env python3
"""
Joycon Publisher Node
发布 joycon 控制器的位姿数据到 ROS2 话题
"""

import rclpy
from rclpy.node import Node
from custom_msgs.msg import JoyconCommand
from std_msgs.msg import Header
from joyconrobotics import JoyconRobotics


class JoyconPublisher(Node):
    """Joycon 发布者节点"""

    def __init__(self):
        super().__init__('joycon_publisher')
        
        # 初始化 joycon 控制器
        self.joycon = JoyconRobotics("right")
        self.get_logger().info("成功连接到右侧 joycon")
        
        # 创建发布者，发布到 joycon_command 话题
        self.publisher_ = self.create_publisher(
            JoyconCommand,
            'joycon_command',
            10
        )
        
        # 创建定时器，以 1000Hz 频率发布（周期 = 1/1000 = 0.001 秒）
        timer_period = 0.001  # 1ms = 1000Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Joycon 发布者节点已启动，发布频率: {1.0/timer_period:.0f} Hz')
    
    def timer_callback(self):
        """定时器回调函数，读取 joycon 数据并发布"""
        try:
            # 获取 joycon 控制数据
            pose, gripper, control_button = self.joycon.get_control()
            
            # 解析位姿数据: [x, y, z, roll, pitch, yaw]
            if len(pose) >= 6:
                x, y, z, roll, pitch, yaw = pose[:6]
                
                # 创建消息
                msg = JoyconCommand()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'joycon_frame'
                
                # 填充位姿数据: [x, y, z, roll, pitch, yaw]
                msg.x_cartesian = [float(x), float(y), float(z), 
                                   float(roll), float(pitch), float(yaw)]
                
                # 发布消息
                self.publisher_.publish(msg)
                
            else:
                self.get_logger().warn(f'收到的位姿数据长度不足: {len(pose)}, 需要 6 个值')
                
        except Exception as e:
            self.get_logger().error(f'读取 joycon 数据时出错: {str(e)}')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        joycon_publisher = JoyconPublisher()
        rclpy.spin(joycon_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点运行出错: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
