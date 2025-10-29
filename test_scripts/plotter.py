
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

import matlplotlib.pyplot as plt

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
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', self.pix_odom, self.qos_profile_1)
        self.odom_msg = VehicleOdometry()

        self.timePeriod = 10
        self.timer =  self.create_timer(self.timePeriod, self.timer_cb)
        self.ouster_odom = []
        self.pixhawk_odom = []

    def publish_odom(self, msg):
        pose_msg = Pose()
        # pose_msg.header = msg.header
        pose_msg.position = msg.pose.pose.position
        pose_msg.orientation = msg.pose.pose.orientation
        
        self.ouster_odom.append(pose_msg)
    
    def pix_odom(self, msg):
        pose_msg = Pose()
        # pose_msg.header.stamp = msg.timestamp
        pose_msg.position.x = msg.position.x
        pose_msg.position.y = -msg.position.y
        pose_msg.position.z = -msg.position.z
        pose_msg.orientation.x = msg.q.x
        pose_msg.orientation.y = -msg.q.y
        pose_msg.orientation.z = -msg.q.z
        pose_msg.orientation.w = msg.q.w
        
        self.pixhawk_odom.append(pose_msg)

    def timer_cb(self):
        xo = []
        yo = []
        zo = []
        xp = []
        yp = []
        zp = []
        for msg in self.ouster_odom:
            xo.append(msg.position.x)
            yo.append(msg.position.y)
            zo.append(msg.position.z)
        for msg in self.pix_odom:
            xp.append(msg.position.x)
            yp.append(msg.position.y)
            zp.append(msg.position.z)

        plt.figure()
        plt.plot(xo)
        plt.plot(xp)
        plt.show()
    def range_cb(self, msg):
        range = msg.range


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
