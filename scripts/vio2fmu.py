
__author__ = "Viswa Narayanan Sankaranarayanan"
__contact__ = "vissan@ltu.se"

import rclpy
import numpy as np
from numpy import nan
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

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, TwistStamped, Vector3
from sensor_msgs.msg import Range

class DlioOdom(Node):
	def __init__(self):
		super().__init__('mocap_to_odom')
		self.qos_profile_1 = QoSProfile(
		reliability=QoSReliabilityPolicy.BEST_EFFORT,
		durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
		history=QoSHistoryPolicy.KEEP_LAST,
		depth=1
		)
		self.qos_profile_2 = QoSProfile(
		reliability=QoSReliabilityPolicy.BEST_EFFORT,
		durability=QoSDurabilityPolicy.VOLATILE,
		history=QoSHistoryPolicy.KEEP_LAST,
		depth=1
		)
		
		self.dlio_odom_sub = self.create_subscription(Odometry, '/shafterx2/odometry/imu', self.publish_odom, self.qos_profile_2)
		self.range_sub = self.create_subscription(Range, '/sbl', self.range_cb, self.qos_profile_2)
		self.odom_pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', self.qos_profile_1)
		self.odom_msg = VehicleOdometry()
		self.quat = np.array([0.0, 0.0, 0.0, 1.0])

		self.rangeFlag = False
		self.lioFlag = False

	def publish_odom(self, msg):
		if not self.lioFlag:
			self.lioFlag = True
			print('Lio messages received. Relaying odometry')
		self.odom_msg.timestamp = timesync(msg.header.stamp)
		self.odom_msg.pose_frame = 1
		if self.rangeFlag:
			self.odom_msg.position[0] = msg.pose.pose.position.x
			self.odom_msg.position[1] = -msg.pose.pose.position.y
		else:
			self.odom_msg.position = [msg.pose.pose.position.x, -msg.pose.pose.position.y, -msg.pose.pose.position.z]
		self.odom_msg.q = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, -msg.pose.pose.orientation.z]
		self.quat = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
		self.odom_msg.velocity_frame = 1
		self.odom_msg.velocity = [nan, nan, nan]
		self.odom_msg.angular_velocity = [nan, nan, nan]
		self.odom_msg.position_variance = [0.0, 0.0, 0.0]
		self.odom_msg.orientation_variance = [0.0, 0.0, 0.0]

		self.odom_pub.publish(self.odom_msg)

	def range_cb(self, msg):
		if not self.rangeFlag:
			self.rangeFlag =  True
			
		if self.lioFlag:
			[roll, pitch, yaw] = euler_from_quaternion(self.quat)
		else:
			roll = 0.0
			pitch = 0.0

		# Add the transformation for the offset in the Single Beam Lidar's position
		self.odom_msg.position[2] = -msg.range*np.cos(roll)*np.cos(pitch)


def timesync(stamp):
	time = stamp.sec*1E9 + stamp.nanosec
	return (int(time/1E3))

        

def main(args=None):
    rclpy.init(args=args)
    dlio_odom = DlioOdom()

    rclpy.spin(dlio_odom)

    dlio_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
