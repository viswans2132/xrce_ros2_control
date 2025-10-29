#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Current velocity
        self.linear_speed = [0.0, 0.0, 0.0]

        self.get_logger().info(
            "Keyboard teleop started. Use arrow keys to move x/y, W/S for z. Press ESC to exit."
        )

        # Start pynput listener
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        # Publish at 20 Hz
        self.create_timer(0.05, self.publish_twist)

    def on_press(self, key):
        try:
            if key.char == 'w':
                self.linear_speed[2] += 0.1  # Up
            elif key.char == 's':
                self.linear_speed[2] -= 0.1  # Down
        except AttributeError:
            # Handle special keys (arrows)
            if key == keyboard.Key.up:
                self.linear_speed[0] += 0.1  # Forward x+
            elif key == keyboard.Key.down:
                self.linear_speed[0] -= 0.1  # Backward x-
            elif key == keyboard.Key.right:
                self.linear_speed[1] -= 0.1  # Right y-
            elif key == keyboard.Key.left:
                self.linear_speed[1] += 0.1  # Left y+
            elif key == keyboard.Key.esc:
                self.get_logger().info("Exiting teleop...")
                rclpy.shutdown()

        self.get_logger().info(
            f"Velocities: x={self.linear_speed[0]:.2f}, y={self.linear_speed[1]:.2f}, z={self.linear_speed[2]:.2f}"
        )

    def publish_twist(self):
        msg = Twist()
        msg.linear.x = self.linear_speed[0]
        msg.linear.y = self.linear_speed[1]
        msg.linear.z = self.linear_speed[2]
        self.publisher_.publish(msg)


def main():
    rclpy.init()
    node = KeyboardTeleop()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
