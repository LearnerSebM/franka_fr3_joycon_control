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
        self.declare_parameter('reset_controller_name', 'robot_reset_controller')
        self.declare_parameter('target_controller_name', 'franka_joycon_controller')
        self.declare_parameter('check_interval', 0.5)  # seconds
        
        namespace = self.get_parameter('namespace').get_parameter_value().string_value
        reset_controller_name = self.get_parameter('reset_controller_name').get_parameter_value().string_value
        target_controller_name = self.get_parameter('target_controller_name').get_parameter_value().string_value
        check_interval = self.get_parameter('check_interval').get_parameter_value().double_value
        
        # Build service names with namespace
        # Controller services are under controller name node
        if namespace:
            reset_status_service = f'/{namespace}/robot_reset_status'
            switch_controller_service = f'/{namespace}/controller_manager/switch_controller'
            load_controller_service = f'/{namespace}/controller_manager/load_controller'
            configure_controller_service = f'/{namespace}/controller_manager/configure_controller'
        else:
            reset_status_service = '/robot_reset_status'
            switch_controller_service = '/controller_manager/switch_controller'
            load_controller_service = '/controller_manager/load_controller'
            configure_controller_service = '/controller_manager/configure_controller'
        
        # Create clients
        self.reset_status_client = self.create_client(Trigger, reset_status_service)
        self.switch_controller_client = self.create_client(SwitchController, switch_controller_service)
        self.load_controller_client = self.create_client(LoadController, load_controller_service)
        self.configure_controller_client = self.create_client(ConfigureController, configure_controller_service)
        
        self.reset_controller_name = reset_controller_name
        self.target_controller_name = target_controller_name
        self.reset_rcv = False  # flag to check if reset_status response is received
        self.switched = False
        
        # Wait for services
        self.get_logger().info(f'Waiting for service: {reset_status_service}')
        self.reset_status_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info(f'Waiting for service: {load_controller_service}')
        self.load_controller_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info(f'Waiting for service: {configure_controller_service}')
        self.configure_controller_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info(f'Waiting for service: {switch_controller_service}')
        self.switch_controller_client.wait_for_service(timeout_sec=10.0)
        
        # Create timer to check reset status
        self.timer = self.create_timer(check_interval, self.check_reset_status)
        self.get_logger().info('SwitchController started, monitoring reset status...')
    
    def check_reset_status(self):
        if self.switched:
            return
        
        request = Trigger.Request()
        future = self.reset_status_client.call_async(request)
        future.add_done_callback(self.reset_status_callback)
    
    def reset_status_callback(self, future):
        try:
            response = future.result()
            if response.success and not self.reset_rcv:
                self.get_logger().info(f'Reset completed (message: {response.message}), loading target controller...')
                time.sleep(1.0)
                # sleep incase the robot is still in "Idle" mode which is not ready to move
                self.reset_rcv = True
                self.load_target_controller()
            else:
                self.get_logger().debug(f'Reset not completed yet (message: {response.message})')
        except Exception as e:
            self.get_logger().error(f'Error checking reset status: {e}')
    
    def load_target_controller(self):
        """Load the target controller before switching."""
        if self.switched:
            return
        
        request = LoadController.Request()
        request.name = self.target_controller_name
        
        self.get_logger().info(f'Loading controller: {self.target_controller_name}')
        future = self.load_controller_client.call_async(request)
        future.add_done_callback(self.load_controller_callback)
    
    def load_controller_callback(self, future):
        """Callback for load controller service."""
        try:
            response = future.result()
            if response.ok:
                self.get_logger().info(f'Successfully loaded {self.target_controller_name}, configuring...')
                self.configure_target_controller()
            else:
                self.get_logger().error(f'Failed to load {self.target_controller_name}: {response}')
        except Exception as e:
            self.get_logger().error(f'Error loading controller: {e}')
    
    def configure_target_controller(self):
        """Configure the target controller before switching."""
        if self.switched:
            return
        
        request = ConfigureController.Request()
        request.name = self.target_controller_name
        
        self.get_logger().info(f'Configuring controller: {self.target_controller_name}')
        future = self.configure_controller_client.call_async(request)
        future.add_done_callback(self.configure_controller_callback)
    
    def configure_controller_callback(self, future):
        """Callback for configure controller service."""
        try:
            response = future.result()
            if response.ok:
                self.get_logger().info(f'Successfully configured {self.target_controller_name}, switching controllers...')
                self.switch_controllers()
            else:
                self.get_logger().error(f'Failed to configure {self.target_controller_name}: {response}')
        except Exception as e:
            self.get_logger().error(f'Error configuring controller: {e}')
    
    def switch_controllers(self):
        if self.switched:
            return
        
        request = SwitchController.Request()
        request.activate_controllers = [self.target_controller_name]
        request.deactivate_controllers = [self.reset_controller_name]
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        timeout_duration = Duration()
        timeout_duration.sec = 5
        timeout_duration.nanosec = 0
        request.timeout = timeout_duration
        
        future = self.switch_controller_client.call_async(request)
        future.add_done_callback(self.switch_controller_callback)
    
    def switch_controller_callback(self, future):
        try:
            response = future.result()
            if response.ok:
                self.get_logger().info('Controllers switched successfully')
                self.switched = True
                self.timer.cancel()
            else:
                self.get_logger().error(f'Failed to switch controllers: {response}')
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

