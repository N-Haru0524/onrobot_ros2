#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from onrobot_interfaces.srv import SetWidthForceSpeed

LEFT_FINGER_JOINT = 'left_finger_joint'
LEFT_FINGER_MAX_TRAVEL_M = 0.019
DEFAULT_FORCE = 100
DEFAULT_SPEED = 80


class OnRobot2FG7GuiBridge(Node):

    def __init__(self):
        super().__init__('onrobot_2fg7_gui_bridge')

        self.declare_parameter('joint_name', LEFT_FINGER_JOINT)
        self.declare_parameter('gui_joint_states_topic', '/onrobot_2fg7/gui_joint_states')
        self.declare_parameter('move_service', '/onrobot_2fg7/move')
        self.declare_parameter('width_min_mm', 33.0)
        self.declare_parameter('width_max_mm', 71.0)
        self.declare_parameter('force', DEFAULT_FORCE)
        self.declare_parameter('speed', DEFAULT_SPEED)
        self.declare_parameter('command_period_sec', 0.25)
        self.declare_parameter('width_epsilon_mm', 0.5)

        self.joint_name = self.get_parameter('joint_name').value
        self.width_min = float(self.get_parameter('width_min_mm').value)
        self.width_max = float(self.get_parameter('width_max_mm').value)
        self.force = int(self.get_parameter('force').value)
        self.speed = int(self.get_parameter('speed').value)
        self.command_period = float(self.get_parameter('command_period_sec').value)
        self.width_epsilon = float(self.get_parameter('width_epsilon_mm').value)
        self._last_sent_width = None
        self._last_command_time = 0.0
        self._pending_width = None
        self._request_in_flight = False
        self._gui_initialized = False
        self._gui_last_width = None

        self.move_client = self.create_client(
            SetWidthForceSpeed,
            self.get_parameter('move_service').value,
        )
        self.gui_joint_sub = self.create_subscription(
            JointState,
            self.get_parameter('gui_joint_states_topic').value,
            self.on_gui_joint_state,
            10,
        )
        self.command_timer = self.create_timer(0.05, self.flush_pending_command)

    def joint_position_to_width(self, joint_position):
        normalized = max(0.0, min(1.0, joint_position / LEFT_FINGER_MAX_TRAVEL_M))
        return self.width_min + normalized * (self.width_max - self.width_min)

    def on_gui_joint_state(self, msg):
        if self.joint_name not in msg.name:
            return

        idx = msg.name.index(self.joint_name)
        joint_position = float(msg.position[idx])
        width = self.joint_position_to_width(joint_position)

        # Ignore the first GUI publish so startup does not move the real gripper
        # to the GUI widget's default position.
        if not self._gui_initialized:
            self._gui_initialized = True
            self._gui_last_width = width
            return

        if self._gui_last_width is not None and math.fabs(width - self._gui_last_width) < self.width_epsilon:
            return

        self._gui_last_width = width

        if self._last_sent_width is not None and math.fabs(width - self._last_sent_width) < self.width_epsilon:
            return

        self._pending_width = width

    def flush_pending_command(self):
        if self._pending_width is None or self._request_in_flight:
            return

        now = time.monotonic()
        if now - self._last_command_time < self.command_period:
            return

        if not self.move_client.wait_for_service(timeout_sec=0.0):
            return

        request = SetWidthForceSpeed.Request()
        request.width = float(self._pending_width)
        request.force = self.force
        request.speed = self.speed
        request.wait = False

        self._request_in_flight = True
        future = self.move_client.call_async(request)
        future.add_done_callback(self.on_move_response)
        self._last_command_time = now
        self._last_sent_width = float(self._pending_width)
        self._pending_width = None

    def on_move_response(self, future):
        self._request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().info(f'GUI bridge move call failed: {exc}')
            return

        if not response.success:
            self.get_logger().info(f'GUI bridge move failed: {response.message}')


def main(args=None):
    rclpy.init(args=args)
    node = OnRobot2FG7GuiBridge()
    rclpy.spin(node)
    node.destroy_timer(node.command_timer)
    node.destroy_subscription(node.gui_joint_sub)
    node.destroy_client(node.move_client)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
