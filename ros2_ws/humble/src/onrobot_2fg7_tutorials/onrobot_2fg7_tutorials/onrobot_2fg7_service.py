#!/usr/bin/env python3

import time
import xmlrpc.client

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

from onrobot_interfaces.srv import SetWidthForceSpeed

CONN_ERR = -2
RET_OK = 0
RET_FAIL = -1

TWOFG_ID = 0xC0
IP_ADDRESS = '192.168.0.4'
LEFT_FINGER_JOINT = 'left_finger_joint'
LEFT_FINGER_MAX_TRAVEL_M = 0.019
DEFAULT_FORCE = 100
DEFAULT_SPEED = 80
DEFAULT_WIDTH_MM = 71.0


class OnRobot2FG7Server(Node):

    def __init__(self):
        super().__init__('onrobot_2fg7_command_server')

        self.cb = xmlrpc.client.ServerProxy(f'http://{IP_ADDRESS}:41414/')
        self.device_index = 0
        self.width_min = 33.0
        self.width_max = 71.0
        self.last_width = DEFAULT_WIDTH_MM
        self._last_connection_log = 0.0

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, '/onrobot_2fg7/joint_states', qos_profile)

        self.move_srv = self.create_service(
            SetWidthForceSpeed,
            '/onrobot_2fg7/move',
            self.onrobot_2fg7_move,
        )
        self.grip_srv = self.create_service(
            SetWidthForceSpeed,
            '/onrobot_2fg7/grip',
            self.onrobot_2fg7_grip,
        )
        self.timer = self.create_timer(0.1, self.publish_joint_state)

        self.refresh_width_limits()
        self.sync_current_width()

    def is_connected(self, t_index=0):
        try:
            is_twofg = self.cb.cb_is_device_connected(t_index, TWOFG_ID)
        except (OSError, xmlrpc.client.Error) as exc:
            now = time.monotonic()
            if now - self._last_connection_log > 5.0:
                self.get_logger().info(f'Connection to Compute Box failed: {exc}')
                self._last_connection_log = now
            return False

        if not is_twofg:
            now = time.monotonic()
            if now - self._last_connection_log > 5.0:
                self.get_logger().info('No 2FG device connected on the given instance')
                self._last_connection_log = now
            return False
        return True

    def is_busy(self, t_index=0):
        if not self.is_connected(t_index):
            return CONN_ERR
        return self.cb.twofg_get_busy(t_index)

    def is_gripped(self, t_index=0):
        if not self.is_connected(t_index):
            return CONN_ERR
        return self.cb.twofg_get_grip_detected(t_index)

    def get_external_width(self, t_index=0):
        if not self.is_connected(t_index):
            return CONN_ERR
        return self.cb.twofg_get_external_width(t_index)

    def get_min_external_width(self, t_index=0):
        if not self.is_connected(t_index):
            return CONN_ERR
        return self.cb.twofg_get_min_external_width(t_index)

    def get_max_external_width(self, t_index=0):
        if not self.is_connected(t_index):
            return CONN_ERR
        return self.cb.twofg_get_max_external_width(t_index)

    def refresh_width_limits(self):
        min_width = self.get_min_external_width(self.device_index)
        max_width = self.get_max_external_width(self.device_index)
        if min_width != CONN_ERR and max_width != CONN_ERR and max_width > min_width:
            self.width_min = float(min_width)
            self.width_max = float(max_width)
            self.last_width = max(self.width_min, min(self.last_width, self.width_max))

    def sync_current_width(self):
        width = self.get_external_width(self.device_index)
        if width != CONN_ERR:
            self.last_width = float(width)

    def validate_width(self, width):
        self.refresh_width_limits()
        if width < self.width_min or width > self.width_max:
            return (
                False,
                f'Invalid 2FG width parameter, valid range: {self.width_min}-{self.width_max} mm',
            )
        return True, ''

    def validate_force(self, force):
        if force < 20 or force > 140:
            return False, 'Invalid 2FG force parameter, valid range: 20-140 N'
        return True, ''

    def validate_speed(self, speed):
        if speed < 10 or speed > 100:
            return False, 'Invalid 2FG speed parameter, valid range: 10-100 %'
        return True, ''

    def move(self, t_index, width, force=DEFAULT_FORCE, speed=DEFAULT_SPEED, wait=True):
        if not self.is_connected(t_index):
            return CONN_ERR

        valid, message = self.validate_width(width)
        if not valid:
            self.get_logger().info(message)
            return RET_FAIL

        valid, message = self.validate_force(force)
        if not valid:
            self.get_logger().info(message)
            return RET_FAIL

        valid, message = self.validate_speed(speed)
        if not valid:
            self.get_logger().info(message)
            return RET_FAIL

        self.cb.twofg_grip_external(t_index, float(width), int(force), int(speed))
        if not wait:
            return RET_OK

        timeout_count = 0
        busy = self.is_busy(t_index)
        while busy is True:
            time.sleep(0.1)
            busy = self.is_busy(t_index)
            timeout_count += 1
            if timeout_count > 30:
                self.get_logger().info('2FG move command timeout')
                return RET_FAIL
        if busy == CONN_ERR:
            return CONN_ERR
        return RET_OK

    def grip(self, t_index, width, force, speed, wait=True):
        result = self.move(t_index, width, force=force, speed=speed, wait=wait)
        if result != RET_OK or not wait:
            return result

        timeout_count = 0
        gripped = self.is_gripped(t_index)
        while gripped is False:
            time.sleep(0.1)
            gripped = self.is_gripped(t_index)
            timeout_count += 1
            if timeout_count > 20:
                self.get_logger().info('2FG grip detection timeout')
                return RET_FAIL
        if gripped == CONN_ERR:
            return CONN_ERR
        return RET_OK

    def width_to_joint_position(self, width):
        if self.width_max <= self.width_min:
            return 0.0

        normalized = (width - self.width_min) / (self.width_max - self.width_min)
        normalized = max(0.0, min(1.0, normalized))
        return (1.0 - normalized) * LEFT_FINGER_MAX_TRAVEL_M

    def publish_joint_state(self):
        width = self.get_external_width(self.device_index)
        if width == CONN_ERR:
            width = self.last_width
        else:
            self.last_width = float(width)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [LEFT_FINGER_JOINT]
        msg.position = [self.width_to_joint_position(float(width))]
        self.joint_pub.publish(msg)

    def fill_response(self, response, result, message=''):
        response.success = (result == RET_OK)
        response.message = '' if response.success else message
        return response

    def onrobot_2fg7_move(self, request, response):
        result = self.move(
            t_index=self.device_index,
            width=request.width,
            force=request.force if request.force > 0 else DEFAULT_FORCE,
            speed=request.speed if request.speed > 0 else DEFAULT_SPEED,
            wait=request.wait,
        )
        message = (
            f'move failed for width={request.width}, force={request.force}, speed={request.speed}'
            if result != RET_OK else ''
        )
        return self.fill_response(response, result, message)

    def onrobot_2fg7_grip(self, request, response):
        result = self.grip(
            t_index=self.device_index,
            width=request.width,
            force=request.force if request.force > 0 else DEFAULT_FORCE,
            speed=request.speed if request.speed > 0 else DEFAULT_SPEED,
            wait=request.wait,
        )
        message = (
            f'grip failed for width={request.width}, force={request.force}, speed={request.speed}'
            if result != RET_OK else ''
        )
        return self.fill_response(response, result, message)


def main(args=None):
    rclpy.init(args=args)
    node = OnRobot2FG7Server()
    rclpy.spin(node)
    node.destroy_timer(node.timer)
    node.destroy_service(node.move_srv)
    node.destroy_service(node.grip_srv)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
