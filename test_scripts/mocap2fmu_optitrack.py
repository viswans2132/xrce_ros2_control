
__author__ = "Viswa Narayanan Sankaranarayanan"
__contact__ = "vissan@ltu.se"

import rclpy
import numpy as np
from numpy import NaN
from tf_transformations import *
#import RPi.GPIO as GPIO
import time
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleOdometry

from custom_msgs.msg import ArucoBoardPosition

from mocap_msgs.msg import RigidBodies

class MocapOdom(Node):
	def __init__(self):
		super().__init__('mocap_to_odom')
		self.qos_profile_1 = QoSProfile(
		reliability=QoSReliabilityPolicy.BEST_EFFORT,
		durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
		history=QoSHistoryPolicy.KEEP_LAST,
		depth=1
		)
		self.qos_profile_2 = QoSProfile(depth=1)
		
		self.rigid_body_sub = self.create_subscription(RigidBodies, '/rigid_bodies', self.publish_odom, self.qos_profile_2)
		self.odom_pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', self.qos_profile_1)
		self.aruco_board_pose_pub = self.create_publisher(ArucoBoardPosition, '/aruco/board_position', self.qos_profile_1)

	def publish_odom(self, msg):
		no_rigid_bodies = len(msg.rigidbodies)
		odom_msg = VehicleOdometry()
		board_pose_msg = ArucoBoardPosition()
		
		if no_rigid_bodies > 0:
			for rigidbody in msg.rigidbodies:
				if rigidbody.rigid_body_name == '24430':
					odom_msg.timestamp = timesync(msg.header.stamp)
					odom_msg.pose_frame = 1
					odom_msg.position = [rigidbody.pose.position.x, -rigidbody.pose.position.y, -rigidbody.pose.position.z]
					odom_msg.q = [rigidbody.pose.orientation.w, rigidbody.pose.orientation.x, rigidbody.pose.orientation.y, -rigidbody.pose.orientation.z]
					odom_msg.velocity_frame = 1
					odom_msg.velocity = [NaN, NaN, NaN]
					odom_msg.angular_velocity = [NaN, NaN, NaN]
					odom_msg.position_variance = [0.0, 0.0, 0.0]
					odom_msg.orientation_variance = [0.0, 0.0, 0.0]
					# self.odom_pub.publish(odom_msg)
					
				if rigidbody.rigid_body_name == '24435':
					board_pose_msg.timestamp = timesync(msg.header.stamp)
					board_pose_msg.position = [rigidbody.pose.position.x, -rigidbody.pose.position.y, -rigidbody.pose.position.z]
					self.aruco_board_pose_pub.publish(board_pose_msg)


def timesync(stamp):
	time = stamp.sec*1E9 + stamp.nanosec
	return (int(time/1E3))

        

def main(args=None):
    rclpy.init(args=args)
    mocap_odom = MocapOdom()

    rclpy.spin(mocap_odom)

    mocap_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
