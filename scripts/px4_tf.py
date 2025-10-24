#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy



class PX4TFBroadcaster(Node):
    def __init__(self):
        super().__init__('px4_tf_broadcaster')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Dynamic TF broadcaster (map -> base_link)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Static TF broadcaster (base_link -> lidar_link)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_lidar_tf()

        # PX4 odometry subscriber
        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',  # topic from microRTPS bridge
            self.odom_callback,
            qos_profile
        )

        self.get_logger().info(
            "PX4 TF broadcaster started: publishing map→base_link (dynamic) and base_link→lidar_link (static)."
        )

    def publish_static_lidar_tf(self):
        """Static transform: base_link → lidar_link."""
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'base_link'
        static_tf.child_frame_id = 'lidar_link'

        # Pose of lidar from Gazebo x500_lidar_2d model
        static_tf.transform.translation.x = 0.12
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 0.26

        # No rotation offset (roll, pitch, yaw = 0)
        # q = quaternion_from_euler(0.0, 0.0, 0.0)
        static_tf.transform.rotation.x = 0.0
        static_tf.transform.rotation.y = 0.0
        static_tf.transform.rotation.z = 0.0
        static_tf.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform(static_tf)
        self.get_logger().info(
            "Published static TF: base_link → lidar_link (x=0.12, y=0.0, z=0.26)"
        )

    def odom_callback(self, msg: VehicleOdometry):
        """Dynamic transform: map → base_link."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        # PX4 uses NED (x-forward, y-right, z-down)
        # but tf uses ENU (x-forward, y-left, z-up)
        # Here we assume PX4 messages are already rotated to ENU
        t.transform.translation.x = float(msg.position[0])
        t.transform.translation.y = -float(msg.position[1])
        t.transform.translation.z = -float(msg.position[2])

        t.transform.rotation.x = float(msg.q[1])
        t.transform.rotation.y = float(msg.q[2])
        t.transform.rotation.z = -float(msg.q[3])
        t.transform.rotation.w = float(msg.q[0])

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PX4TFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
