
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
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, TwistStamped, Vector3, TransformStamped
from sensor_msgs.msg import Range
from tf2_ros import TransformBroadcaster

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
		
		self.dlio_odom_sub = self.create_subscription(Odometry, '/ov_msckf/odomimu', self.publish_odom, self.qos_profile_2)
		self.range_sub = self.create_subscription(Range, '/sbl', self.range_cb, self.qos_profile_2)
		self.odom_pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', self.qos_profile_1)
		self.veh_odom_msg = VehicleOdometry()
		self.quat = np.array([0.0, 0.0, 0.0, 1.0])
		self.dlio_odom_pub = self.create_publisher(Odometry, '/odom', self.qos_profile_2)
		self.odom_msg = Odometry()

		self.rangeFlag = False
		self.lioFlag = False

	def publish_odom(self, msg):
		if not self.lioFlag:
			self.lioFlag = True
			print('Lio messages received. Relaying odometry')



		# Pose of Pixhawk's center in the camera frame
		t_matrix = np.array([[ 0.0,  -1.0,  0.0, 0.0], #0.13
								[0.0,  0.0,  -1.0, -0.04], 
								[ 1.0,  0.0,  0.0, -0.2], #-0.04
								[ 0.0,  0.0,  0.0, 1.0]])

		# Camera's pose in the global frame
		cam_transform = quaternion_matrix([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
		cam_transform[0,-1] = msg.pose.pose.position.x
		cam_transform[1,-1] = msg.pose.pose.position.y
		cam_transform[2,-1] = msg.pose.pose.position.z

		# UAV's pose in the global frame
		uav_transform = cam_transform@t_matrix;

		# RViz Odometry Visualization with an offset in Z axis
		self.odom_msg.header = msg.header
		self.odom_msg.pose.pose.position.x = uav_transform[0,-1]
		self.odom_msg.pose.pose.position.y = uav_transform[1,-1]
		self.odom_msg.pose.pose.position.z = uav_transform[2,-1] + 1.5
		quat = quaternion_from_matrix(uav_transform)
		self.odom_msg.pose.pose.orientation.x = quat[0]
		self.odom_msg.pose.pose.orientation.y = quat[1]
		self.odom_msg.pose.pose.orientation.z = quat[2]
		self.odom_msg.pose.pose.orientation.w = quat[3]

		self.dlio_odom_pub.publish(self.odom_msg)

		# # RViz TF Visualization with an offset in Z axis		
		# t = TransformStamped()
		# t.header = msg.header
		# t.child_frame_id = 'uav'
		# t.transform.translation.x = uav_transform[0,-1]
		# t.transform.translation.y = uav_transform[1,-1]
		# t.transform.translation.z = uav_transform[0,-1] + 1.5

		# t.transform.rotation.x = quat[0]
		# t.transform.rotation.y = quat[1]
		# t.transform.rotation.z = quat[2]
		# t.transform.rotation.w = quat[3]

		# br = TransformBroadcaster(self)
		# br.sendTransform(t)

		self.veh_odom_msg.timestamp = timesync(msg.header.stamp)
		self.veh_odom_msg.pose_frame = 1
		if self.rangeFlag:
			self.veh_odom_msg.position[0] = uav_transform[0, -1]
			self.veh_odom_msg.position[1] = -uav_transform[1, -1]
		else:
			self.veh_odom_msg.position = [uav_transform[0, -1], -uav_transform[1, -1], -uav_transform[2, -1]]
		self.veh_odom_msg.q = [quat[3], quat[0], quat[1], -quat[2]]
		self.quat = quat
		
		self.veh_odom_msg.velocity_frame = 1
		self.veh_odom_msg.velocity = [nan, nan, nan]
		self.veh_odom_msg.angular_velocity = [nan, nan, nan]
		self.veh_odom_msg.position_variance = [0.0, 0.0, 0.0]
		self.veh_odom_msg.orientation_variance = [0.0, 0.0, 0.0]

		self.odom_pub.publish(self.veh_odom_msg)

	def range_cb(self, msg):
		if not self.rangeFlag:
			self.rangeFlag =  True
			# pass
		range = msg.range
		if self.lioFlag:
			[roll, pitch, yaw] = euler_from_quaternion(self.quat)
		else:
			roll = 0.0
			pitch = 0.0
		self.veh_odom_msg.position[2] = -msg.range*np.cos(roll)*np.cos(pitch)


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
