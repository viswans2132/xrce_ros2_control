#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

class CameraTopicBridge(Node):
    def __init__(self):
        super().__init__('camera_topic_bridge')

        # Subscribe to the topics published by ros_gz_image
        self.image_sub = self.create_subscription(
            Image,
            '/world/default/model/quadtailsitter_0/link/camera_link/sensor/imager/image',
            self.image_callback,
            10
        )
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/world/default/model/quadtailsitter_0/link/camera_link/sensor/imager/camera_info',
            self.info_callback,
            10
        )

        # Publish to your desired ROS 2 topics
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        self.get_logger().info('Camera bridge node started!')

    def image_callback(self, msg: Image):
        # Simply republish the incoming message
        self.image_pub.publish(msg)

    def info_callback(self, msg: CameraInfo):
        # Simply republish the incoming message
        self.info_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraTopicBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
