#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan as lsr
import gz.transport13 as gz_transport  # may be gz.transport14 on some systems
from gz.msgs10.laserscan_pb2 import LaserScan as lsg
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


class GzLaserBridge(Node):
    def __init__(self):
        laser_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        super().__init__('gz_laser_bridge')

        # ROS publisher
        self.pub = self.create_publisher(lsr, '/scan', laser_qos)

        # Initialize Gazebo Transport node
        self.gz_node = gz_transport.Node()
        topic = '/world/obstacles/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan'

        # Subscribe to the Gazebo topic
        self.gz_node.subscribe(lsg, topic, self.lidar_callback)
        self.get_logger().info(f"Subscribed to Gazebo topic: {topic}")

        # Store sim time reference
        self.start_time_ros = self.get_clock().now().nanoseconds

    def lidar_callback(self, gz_msg):
        """
        Convert gz.msgs.LaserScan → ROS2 LaserScan
        """
        msg = lsr()
        msg.header.frame_id = "lidar_link"

        # Use sim time if available
        now_ros = self.get_clock().now()
        msg.header.stamp = now_ros.to_msg()

        msg.angle_min = gz_msg.angle_min
        msg.angle_max = gz_msg.angle_max
        msg.angle_increment = gz_msg.angle_step
        msg.time_increment = 0.0
        msg.scan_time = 0.05  # example: 20Hz
        msg.range_min = gz_msg.range_min
        msg.range_max = gz_msg.range_max

        # Copy ranges and intensities
        msg.ranges = list(gz_msg.ranges)
        msg.intensities = list(gz_msg.intensities)

        self.pub.publish(msg)
        self.get_logger().debug(f"Published LaserScan with {len(msg.ranges)} points")

def main(args=None):
    rclpy.init(args=args)
    node = GzLaserBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
