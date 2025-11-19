#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
import gz.transport14 as gz  # Use gz.transport12 if you’re using Fortress
import gz.msgs.image_pb2 as image_pb2
import gz.msgs.camera_info_pb2 as camera_info_pb2
import numpy as np

# For image conversion
from cv_bridge import CvBridge
import cv2


class GzCameraBridge(Node):
    def __init__(self):
        super().__init__('gz_camera_bridge')

        # ROS2 publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # CV bridge for image conversion
        self.bridge = CvBridge()

        # Create Gazebo transport node
        self.gz_node = gz.Node()

        # Subscribe to Gazebo topics
        # ✅ Include the message type for each subscription
        self.gz_node.subscribe(
            '/world/default/model/quadtailsitter_0/link/camera_link/sensor/imager/image',
            image_pb2,
            self.gazebo_image_callback
        )
        self.gz_node.subscribe(
            '/world/default/model/quadtailsitter_0/link/camera_link/sensor/imager/camera_info',
            camera_info_pb2,
            self.gazebo_camera_info_callback
        )
        self.get_logger().info('✅ Gz → ROS2 camera bridge node started.')

    def gazebo_camera_info_callback(self, msg):
        """Convert gz.msgs.CameraInfo → sensor_msgs/CameraInfo"""
        info_msg = CameraInfo()
        info_msg.width = msg.width
        info_msg.height = msg.height
        info_msg.k = msg.k  # Intrinsic matrix
        info_msg.p = msg.p  # Projection matrix
        info_msg.d = list(msg.distortion)

        self.info_pub.publish(info_msg)

    def gazebo_image_callback(self, msg):
        """Convert gz.msgs.Image → sensor_msgs/Image"""
        # Extract image parameters
        height = msg.height
        width = msg.width
        step = msg.step
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if msg.pixel_format == msg.RGB_INT8:
            img = data.reshape((height, width, 3))
            encoding = 'rgb8'
        elif msg.pixel_format == msg.BGR_INT8:
            img = data.reshape((height, width, 3))
            encoding = 'bgr8'
        elif msg.pixel_format == msg.L_INT8:
            img = data.reshape((height, width))
            encoding = 'mono8'
        else:
            self.get_logger().warn(f'Unsupported pixel format: {msg.pixel_format}')
            return

        ros_img = self.bridge.cv2_to_imgmsg(img, encoding)
        ros_img.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(ros_img)


def main(args=None):
    rclpy.init(args=args)
    node = GzCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
