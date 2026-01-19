#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController, LoadController, ConfigureController
from builtin_interfaces.msg import Duration
import time


class SwitchControllerNode(Node):
    def __init__(self):
        super().__init__('switch_controller')
        
        # Declare parameters
        self.declare_parameter('namespace', '')
        self.declare_parameter('old_controller', 'robot_reset_controller')
        self.declare_parameter('new_controller', 'franka_joycon_controller')
        self.declare_parameter('check_time', 0.5)  # seconds
        
        namespace = self.get_parameter('namespace').get_parameter_value().string_value
        old_controller = self.get_parameter('old_controller').get_parameter_value().string_value
        new_controller = self.get_parameter('new_controller').get_parameter_value().string_value
        check_time = self.get_parameter('check_time').get_parameter_value().double_value
        
        # build service names with namespace
        if namespace:
            reset_status_srv = f'/{namespace}/robot_reset_status'
            load_srv = f'/{namespace}/controller_manager/load_controller'
            configure_srv = f'/{namespace}/controller_manager/configure_controller'
            switch_srv = f'/{namespace}/controller_manager/switch_controller'
        else:
            reset_status_srv = '/robot_reset_status'
            load_srv = '/controller_manager/load_controller'
            configure_srv = '/controller_manager/configure_controller'
            switch_srv = '/controller_manager/switch_controller'
        
        # create clients
        self.reset_status_client = self.create_client(Trigger, reset_status_srv)
        self.load_cli = self.create_client(LoadController, load_srv)
        self.configure_cli = self.create_client(ConfigureController, configure_srv)
        self.switch_cli = self.create_client(SwitchController, switch_srv)
        
        self.old_controller = old_controller
        self.new_controller = new_controller
        self.reset_rcv = False  # flag to check if reset_status response is received
        self.switched = False
        
        # wait for services
        self.get_logger().info(f'Waiting for service: {reset_status_srv}')
        self.reset_status_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info(f'Waiting for service: {load_srv}')
        self.load_cli.wait_for_service(timeout_sec=10.0)
        self.get_logger().info(f'Waiting for service: {configure_srv}')
        self.configure_cli.wait_for_service(timeout_sec=10.0)
        self.get_logger().info(f'Waiting for service: {switch_srv}')
        self.switch_cli.wait_for_service(timeout_sec=10.0)
        
        # create timer to check reset status
        self.timer = self.create_timer(check_time, self.check_reset_status)
        self.get_logger().info('SwitchController started, monitoring reset status...')
    
    def check_reset_status(self):
        if self.switched:
            return
        
        req = Trigger.Request()
        future = self.reset_cli.call_async(req)
        future.add_done_callback(self.reset_callback)
    
    def reset_callback(self, future):
        try:
            rsp = future.result()
            if rsp.success and not self.reset_rcv:
                self.get_logger().info(f'Reset completed (message: {rsp.message}), loading target controller...')
                time.sleep(1.0)
                # sleep incase the robot is still in "Idle" mode which is not ready to move
                self.reset_rcv = True
                self.load_new_controller()
            else:
                self.get_logger().debug(f'Reset not completed yet (message: {rsp.message})')
        except Exception as e:
            self.get_logger().error(f'Error checking reset status: {e}')
    
    def load_new_controller(self):
        if self.switched:
            return
        
        req = LoadController.Request()
        req.name = self.new_controller
        
        self.get_logger().info(f'Loading controller: {self.new_controller}')
        future = self.load_cli.call_async(req)
        future.add_done_callback(self.load_callback)
    
    def load_callback(self, future):
        try:
            rsp = future.result()
            if rsp.ok:
                self.get_logger().info(f'Successfully loaded {self.new_controller}, configuring...')
                self.configure_new_controller()
            else:
                self.get_logger().error(f'Failed to load {self.new_controller}: {rsp}')
        except Exception as e:
            self.get_logger().error(f'Error loading controller: {e}')
    
    def configure_new_controller(self):
        if self.switched:
            return
        
        req = ConfigureController.Request()
        req.name = self.new_controller
        
        self.get_logger().info(f'Configuring controller: {self.new_controller}')
        future = self.configure_cli.call_async(req)
        future.add_done_callback(self.configure_callback)
    
    def configure_callback(self, future):
        try:
            rsp = future.result()
            if rsp.ok:
                self.get_logger().info(f'Successfully configured {self.new_controller}, switching controllers...')
                self.switch_controllers()
            else:
                self.get_logger().error(f'Failed to configure {self.new_controller}: {rsp}')
        except Exception as e:
            self.get_logger().error(f'Error configuring controller: {e}')
    
    def switch_controllers(self):
        if self.switched:
            return
        
        req = SwitchController.Request()
        req.activate_controllers = [self.new_controller]
        req.deactivate_controllers = [self.old_controller]
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        timeout_duration = Duration()
        timeout_duration.sec = 5
        timeout_duration.nanosec = 0
        req.timeout = timeout_duration
        
        future = self.switch_cli.call_async(req)
        future.add_done_callback(self.switch_callback)
    
    def switch_callback(self, future):
        try:
            rsp = future.result()
            if rsp.ok:
                self.get_logger().info('Controllers switched successfully')
                self.switched = True
                self.timer.cancel()
            else:
                self.get_logger().error(f'Failed to switch controllers: {rsp}')
        except Exception as e:
            self.get_logger().error(f'Error switching controllers: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SwitchControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

