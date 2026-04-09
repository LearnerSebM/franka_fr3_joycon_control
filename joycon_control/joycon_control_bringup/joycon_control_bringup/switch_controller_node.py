#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from controller_manager_msgs.srv import SwitchController, LoadController, ConfigureController
from custom_msgs.msg import JoyconCommand
from builtin_interfaces.msg import Duration
import time


class SwitchControllerNode(Node):

    # Internal state machine constants (self.state)
    STATE_WAIT_RESET_DONE_STARTUP = 'WAIT_RESET_DONE_STARTUP'
    STATE_RUNNING = 'RUNNING'
    STATE_SWITCH_TO_RESET = 'SWITCH_TO_RESET'
    STATE_WAIT_RESET_DONE_RUNTIME = 'WAIT_RESET_DONE_RUNTIME'

    # Handshake phase constants published to joycon_publisher
    PHASE_TOPIC = 'controller_phase'
    PHASE_RUNNING = 'RUNNING'
    PHASE_SWITCHING_TO_RESET = 'SWITCHING_TO_RESET'
    PHASE_IN_RESET = 'IN_RESET'
    PHASE_SWITCHING_TO_TELEOP = 'SWITCHING_TO_TELEOP'

    def __init__(self):
        super().__init__('switch_controller')

        self.declare_parameter('namespace', '')
        self.declare_parameter('reset_controller', 'robot_reset_controller')
        self.declare_parameter('new_controller', 'franka_joycon_controller')
        self.declare_parameter('check_time', 0.5)

        namespace = self.get_parameter('namespace').get_parameter_value().string_value
        reset_controller = self.get_parameter('reset_controller').get_parameter_value().string_value
        new_controller = self.get_parameter('new_controller').get_parameter_value().string_value
        check_time = self.get_parameter('check_time').get_parameter_value().double_value

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

        self.reset_status_cli = self.create_client(Trigger, reset_status_srv)
        self.load_cli = self.create_client(LoadController, load_srv)
        self.configure_cli = self.create_client(ConfigureController, configure_srv)
        self.switch_cli = self.create_client(SwitchController, switch_srv)
        self.phase_pub = self.create_publisher(String, self.PHASE_TOPIC, 10)

        self.reset_controller = reset_controller
        self.new_controller = new_controller
        self.managed_controllers = [self.new_controller]

        self.state = self.STATE_WAIT_RESET_DONE_STARTUP
        self.switch_in_progress = False
        self.last_reset_request = False

        self.get_logger().info(f'Waiting for service: {reset_status_srv}')
        self.reset_status_cli.wait_for_service(timeout_sec=10.0)
        self.get_logger().info(f'Waiting for service: {load_srv}')
        self.load_cli.wait_for_service(timeout_sec=10.0)
        self.get_logger().info(f'Waiting for service: {configure_srv}')
        self.configure_cli.wait_for_service(timeout_sec=10.0)
        self.get_logger().info(f'Waiting for service: {switch_srv}')
        self.switch_cli.wait_for_service(timeout_sec=10.0)

        # subscribe joycon command to capture reset command
        self.joycon_sub = self.create_subscription(
            JoyconCommand,
            'joycon_command',
            self.joycon_command_callback,
            10,
        )

        self.timer = self.create_timer(check_time, self.main_loop)
        self.get_logger().info('SwitchController state machine started')

    def publish_phase(self, phase):
        """
        Publish handshake phase to /<namespace>/controller_phase for joycon_publisher.
        Phases:
        - PHASE_SWITCHING_TO_RESET: switching to reset controller; joycon should hold xyz commands.
        - PHASE_IN_RESET: reset controller is active; joycon can safely rebase xyz bias.
        - PHASE_SWITCHING_TO_TELEOP: switching back to teleoperation controller.
        - PHASE_RUNNING: teleoperation controller is active and normal command streaming resumes.

        This publisher is for hadshake with joycon_publisher node in the joycon_wrapper package.
        Since the joycon_publisher will reset traslation commands when robot is resetting position,
        the joycon_publisher node needs to know current switching phase from handshakes.
        If we don't publish phase, the robot will received control commands with excessice variation 
        due to reset of joycon commands, and stop out of control exception from libfranka.
        """
        msg = String()
        msg.data = phase
        self.phase_pub.publish(msg)

    def joycon_command_callback(self, msg):
        """
        callback function for topic /<namespace>/joycon_command
        """
        reset_request = bool(msg.reset_request)

        if reset_request and not self.last_reset_request:
            if self.state == self.STATE_RUNNING and not self.switch_in_progress:
                self.get_logger().info('Received reset request from JoyCon "+" button')
                self.state = self.STATE_SWITCH_TO_RESET
                self.publish_phase(self.PHASE_SWITCHING_TO_RESET)
            else:
                self.get_logger().debug(
                    f'Ignore reset request in state={self.state}, switch_in_progress={self.switch_in_progress}'
                )

        self.last_reset_request = reset_request

    def main_loop(self):
        """
        Main loop of the controller switch state machine (timer-driven).
        Internal states (self.state):
        - STATE_WAIT_RESET_DONE_STARTUP: startup stage, poll robot_reset_status until reset finishes.
        - STATE_RUNNING: normal teleoperation stage; "+" edge is handled in joycon callback.
        - STATE_SWITCH_TO_RESET: runtime reset requested, switch active controller to robot_reset_controller.
        - STATE_WAIT_RESET_DONE_RUNTIME: after switching to reset controller, poll reset status again.
        Transitions:
        STATE_WAIT_RESET_DONE_STARTUP -> switch back to teleop -> STATE_RUNNING;
        STATE_RUNNING + "+" -> STATE_SWITCH_TO_RESET -> STATE_WAIT_RESET_DONE_RUNTIME
        -> switch back to teleop -> STATE_RUNNING.
        """
        if self.switch_in_progress:
            return

        if self.state in (self.STATE_WAIT_RESET_DONE_STARTUP, self.STATE_WAIT_RESET_DONE_RUNTIME):
            self.check_reset_status()
        elif self.state == self.STATE_SWITCH_TO_RESET:
            self.switch_to_reset_controller()

    def check_reset_status(self):
        req = Trigger.Request()
        future = self.reset_status_cli.call_async(req)
        future.add_done_callback(self.reset_status_callback)

    def reset_status_callback(self, future):
        try:
            rsp = future.result()
        except Exception as e:
            self.get_logger().error(f'Error checking reset status: {e}')
            return

        if rsp.success:
            self.get_logger().info(f'Reset completed (message: {rsp.message}), switching back to target controller')
            time.sleep(1.0)
            if self.state == self.STATE_WAIT_RESET_DONE_STARTUP:
                # initial startup: franka_joycon_controller not loaded/configured yet.
                self.publish_phase(self.PHASE_SWITCHING_TO_TELEOP)
                self.load_new_controller()
            else:
                # runtime reset path: controller is already loaded/configured, only reactivate it.
                self.switch_in_progress = True
                self.publish_phase(self.PHASE_SWITCHING_TO_TELEOP)
                self.switch_from_reset_to_targets()
        else:
            self.get_logger().debug(f'Reset not completed yet (message: {rsp.message})')

    def load_new_controller(self):
        self.switch_in_progress = True
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
                self.switch_in_progress = False
        except Exception as e:
            self.get_logger().error(f'Error loading controller: {e}')
            self.switch_in_progress = False

    def configure_new_controller(self):
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
                self.switch_from_reset_to_targets()
            else:
                self.get_logger().error(f'Failed to configure {self.new_controller}: {rsp}')
                self.switch_in_progress = False
        except Exception as e:
            self.get_logger().error(f'Error configuring controller: {e}')
            self.switch_in_progress = False

    def switch_from_reset_to_targets(self):
        req = SwitchController.Request()
        req.activate_controllers = self.managed_controllers
        req.deactivate_controllers = [self.reset_controller]
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        timeout_duration = Duration()
        timeout_duration.sec = 5
        timeout_duration.nanosec = 0
        req.timeout = timeout_duration

        future = self.switch_cli.call_async(req)
        future.add_done_callback(self.switch_to_target_callback)

    def switch_to_target_callback(self, future):
        try:
            rsp = future.result()
            if rsp.ok:
                self.get_logger().info('Controllers switched successfully -> RUNNING')
                self.state = self.STATE_RUNNING
                self.publish_phase(self.PHASE_RUNNING)
            else:
                self.get_logger().error(f'Failed to switch controllers: {rsp}')
        except Exception as e:
            self.get_logger().error(f'Error switching controllers: {e}')
        finally:
            self.switch_in_progress = False

    def switch_to_reset_controller(self):
        self.switch_in_progress = True
        req = SwitchController.Request()
        req.activate_controllers = [self.reset_controller]
        req.deactivate_controllers = self.managed_controllers
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        timeout_duration = Duration()
        timeout_duration.sec = 5
        timeout_duration.nanosec = 0
        req.timeout = timeout_duration

        self.get_logger().info(
            f'Switching to reset controller, deactivate={self.managed_controllers}, '
            f'activate={[self.reset_controller]}'
        )
        future = self.switch_cli.call_async(req)
        future.add_done_callback(self.switch_to_reset_callback)

    def switch_to_reset_callback(self, future):
        try:
            rsp = future.result()
            if rsp.ok:
                self.get_logger().info('Switched to reset controller, waiting reset completion')
                self.state = self.STATE_WAIT_RESET_DONE_RUNTIME
                self.publish_phase(self.PHASE_IN_RESET)
            else:
                self.get_logger().error(f'Failed to switch to reset controller: {rsp}')
                self.state = self.STATE_RUNNING
                self.publish_phase(self.PHASE_RUNNING)
        except Exception as e:
            self.get_logger().error(f'Error switching to reset controller: {e}')
            self.state = self.STATE_RUNNING
            self.publish_phase(self.PHASE_RUNNING)
        finally:
            self.switch_in_progress = False


def main(args=None):
    rclpy.init(args=args)
    node = SwitchControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

