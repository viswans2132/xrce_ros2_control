#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # Publisher for velocity commands
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Keyboard Teleop started. Use WSAD to move, Q/E to turn, X to stop, CTRL+C to quit.")

        self.settings = termios.tcgetattr(sys.stdin)
        self.linear_speed = 0.5
        self.angular_speed = 1.0

        self.run()

    def get_key(self):
        """Get a single keypress from stdin (non-blocking)."""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        twist = Twist()

        while rclpy.ok():
            key = self.get_key()

            if key.lower() == 'w':
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
            elif key.lower() == 's':
                twist.linear.x = -self.linear_speed
                twist.angular.z = 0.0
            elif key.lower() == 'a':
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed
            elif key.lower() == 'd':
                twist.linear.x = 0.0
                twist.angular.z = -self.angular_speed
            elif key.lower() == 'x':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif key.lower() == 'q':
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed * 1.5
            elif key.lower() == 'e':
                twist.linear.x = 0.0
                twist.angular.z = -self.angular_speed * 1.5
            elif key == '\x03':  # CTRL+C
                break
            else:
                continue

            self.pub.publish(twist)
            self.get_logger().info(
                f"Published cmd_vel: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}"
            )

        twist = Twist()  # Stop before exiting
        self.pub.publish(twist)
        self.get_logger().info("Keyboard teleop stopped. Robot halted.")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
