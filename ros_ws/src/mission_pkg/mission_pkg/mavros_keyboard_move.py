#!/usr/bin/env python3

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode


class MavrosKeyboardMoveNode(Node):
    def __init__(self):
        super().__init__('mavros_keyboard_move_node')

        self.declare_parameter('speed_xy', 0.12)
        self.declare_parameter('speed_z', 0.08)
        self.declare_parameter('speed_yaw', 0.15)
        self.declare_parameter('auto_offboard', True)

        self.speed_xy = float(self.get_parameter('speed_xy').value)
        self.speed_z = float(self.get_parameter('speed_z').value)
        self.speed_yaw = float(self.get_parameter('speed_yaw').value)
        self.auto_offboard = bool(self.get_parameter('auto_offboard').value)

        self.current_state = State()

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )

        self.vel_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )

        self.set_mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode'
        )

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0

        self.counter = 0
        self.offboard_requested = False

        # 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('MAVROS keyboard velocity move node started')
        self.get_logger().info('First: commander takeoff. Then run this node.')
        self.print_help()

    def print_help(self):
        print('')
        print('================ Keyboard Velocity Control ================')
        print('w : forward')
        print('s : backward')
        print('a : left')
        print('d : right')
        print('r : up')
        print('f : down')
        print('q : yaw left')
        print('e : yaw right')
        print('x : stop')
        print('o : request OFFBOARD mode')
        print('h : help')
        print('Ctrl+C : exit')
        print('===========================================================')
        print('')

    def state_callback(self, msg):
        self.current_state = msg

    def get_key(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0.0)
        if dr:
            return sys.stdin.read(1)
        return None

    def request_offboard(self):
        if not self.set_mode_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('/mavros/set_mode service not available')
            return

        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'

        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.offboard_callback)

        self.get_logger().info('Requesting OFFBOARD mode...')

    def offboard_callback(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'OFFBOARD result: mode_sent={result.mode_sent}')
        except Exception as e:
            self.get_logger().error(f'OFFBOARD request failed: {e}')

    def timer_callback(self):
        key = self.get_key()

        if key is not None:
            if key == 'w':
                self.vx = self.speed_xy
                self.vy = 0.0
                self.vz = 0.0
                self.yaw_rate = 0.0
                self.get_logger().info('forward')

            elif key == 's':
                self.vx = -self.speed_xy
                self.vy = 0.0
                self.vz = 0.0
                self.yaw_rate = 0.0
                self.get_logger().info('backward')

            elif key == 'a':
                self.vx = 0.0
                self.vy = self.speed_xy
                self.vz = 0.0
                self.yaw_rate = 0.0
                self.get_logger().info('left')

            elif key == 'd':
                self.vx = 0.0
                self.vy = -self.speed_xy
                self.vz = 0.0
                self.yaw_rate = 0.0
                self.get_logger().info('right')

            elif key == 'r':
                self.vx = 0.0
                self.vy = 0.0
                self.vz = self.speed_z
                self.yaw_rate = 0.0
                self.get_logger().info('up')

            elif key == 'f':
                self.vx = 0.0
                self.vy = 0.0
                self.vz = -self.speed_z
                self.yaw_rate = 0.0
                self.get_logger().info('down')

            elif key == 'q':
                self.vx = 0.0
                self.vy = 0.0
                self.vz = 0.0
                self.yaw_rate = self.speed_yaw
                self.get_logger().info('yaw left')

            elif key == 'e':
                self.vx = 0.0
                self.vy = 0.0
                self.vz = 0.0
                self.yaw_rate = -self.speed_yaw
                self.get_logger().info('yaw right')

            elif key == 'x':
                self.vx = 0.0
                self.vy = 0.0
                self.vz = 0.0
                self.yaw_rate = 0.0
                self.get_logger().info('stop')

            elif key == 'o':
                self.request_offboard()

            elif key == 'h':
                self.print_help()

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.twist.linear.x = self.vx
        msg.twist.linear.y = self.vy
        msg.twist.linear.z = self.vz
        msg.twist.angular.z = self.yaw_rate

        # 핵심: 항상 setpoint를 20Hz로 계속 보내야 OFFBOARD가 유지됨
        self.vel_pub.publish(msg)

        self.counter += 1

        # 처음에는 zero velocity setpoint를 충분히 보낸 뒤 OFFBOARD 요청
        if self.auto_offboard and not self.offboard_requested:
            if self.counter > 80:
                if self.current_state.mode != 'OFFBOARD':
                    self.request_offboard()
                self.offboard_requested = True

        if self.counter % 40 == 0:
            self.get_logger().info(
                f'connected={self.current_state.connected}, '
                f'mode={self.current_state.mode}, '
                f'armed={self.current_state.armed}, '
                f'vx={self.vx:.2f}, vy={self.vy:.2f}, vz={self.vz:.2f}, yaw_rate={self.yaw_rate:.2f}'
            )


def main(args=None):
    old_settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = MavrosKeyboardMoveNode()

    try:
        tty.setcbreak(sys.stdin.fileno())
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()