#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

laser_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)

class FakeLaserPublisher(Node):
    def __init__(self):
        super().__init__('fake_laser_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', qos_profile=laser_qos)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Laser parameters
        self.angle_min = -np.pi/2        # -90 deg
        self.angle_max = np.pi/2         # +90 deg
        self.angle_increment = np.deg2rad(1.0)  # 1 degree resolution
        self.range_min = 0.05
        self.range_max = 5.0

        # Precompute angles
        self.angles = np.arange(self.angle_min, self.angle_max, self.angle_increment)

        # Cube position and size
        self.cube_center = np.array([1.0, 0.0])   # (x,y)
        self.cube_size = 1.0  # unit cube (1x1)
        self.cube_min = self.cube_center - self.cube_size/2
        self.cube_max = self.cube_center + self.cube_size/2

    def timer_callback(self):
        scan_msg = LaserScan()
        scan_msg.header.frame_id = "map"
        scan_msg.header.stamp = self.get_clock().now().to_msg()

        scan_msg.angle_min = float(self.angle_min)
        scan_msg.angle_max = float(self.angle_max)
        scan_msg.angle_increment = float(self.angle_increment)
        scan_msg.range_min = float(self.range_min)
        scan_msg.range_max = float(self.range_max)

        ranges = []
        for angle in self.angles:
            r = self.cast_ray(angle)
            ranges.append(r)

        scan_msg.ranges = ranges
        self.publisher_.publish(scan_msg)
        self.get_logger().info("Published fake LaserScan with cube obstacle.")

    def cast_ray(self, angle):
        """
        Ray-box intersection in 2D (laser at origin, ray = (cosθ, sinθ)).
        Returns distance if hit, else inf.
        """
        direction = np.array([np.cos(angle), np.sin(angle)])
        origin = np.array([0.0, 0.0])

        # Avoid division by zero
        inv_dir = 1.0 / np.where(direction != 0, direction, 1e-9)

        tmin = (self.cube_min - origin) * inv_dir
        tmax = (self.cube_max - origin) * inv_dir

        t1 = np.minimum(tmin, tmax)
        t2 = np.maximum(tmin, tmax)

        t_enter = np.max(t1)
        t_exit = np.min(t2)

        if t_enter < 0 or t_enter > t_exit:
            return float('inf')  # no hit
        return float(t_enter)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLaserPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
