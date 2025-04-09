#!/usr/bin/env python

__author__ = "Viswa Narayanan Sankaranarayanan"
__contact__ = "vissan@ltu.se"

import rclpy
import numpy as np
from tf_transformations import *
import time
import mavros
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry, VehicleAttitudeSetpoint, VehicleControlMode, VehicleLocalPosition
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import Thrust, State
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, TwistStamped, Vector3
from std_msgs.msg import Int8


class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        qos_profile_transient = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_profile_volatile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_profile_volatile_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Nodes
        self.node = rclpy.create_node('control_class')

        # Subscribers
        self.odomSub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile_transient)
        self.relaySub = self.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', self.relay_callback, qos_profile_transient)
        self.posSpSub = self.create_subscription(PoseStamped, '/shafterx2/reference', self.sp_callback, qos_profile_volatile)
        
        #Publishers
        self.posSpPub = self.create_publisher(PoseStamped, '/ref', qos_profile_volatile)


        timerPeriod = 0.02  # seconds
        self.timer = self.create_timer(timerPeriod, self.cmdloop_callback)


        # initial values for setpoints
        self.curPos = np.zeros((3,))
        self.curVel = np.zeros((3,))
        self.yaw = 0.0
        self.startYaw = 1.0

        # Setpoints
        self.posSp = np.array([1.0,1.0, 1.0])
        self.quatSp = np.array([0.0, 0.0, 0.0, 1.0])
        self.velSp = np.array([0.0,0.0,0.0])
        self.yawSp = 1.5

        # Time parameters
        self.preTime = Clock().now().nanoseconds/1E9
        self.offboardTime = Clock().now().nanoseconds/1E9
        self.dt = timerPeriod
        self.offbCounter = 1

        # Msg Variables
        self.posSpMsg = PoseStamped()


        # Flags
        self.offbFlag = False
        self.armFlag = False
        self.missionFlag = False
        self.relayFlag = False
        self.odomFlag = False
        self.trajFlag = False
        self.home = False
        self.state = State()

        self.takeoffThreshold = 0.75

        
        print("Sleeping")
        time.sleep(2)
        print("Awake")

        

    def relay_callback(self, msg):
        if self.relayFlag == False:
            self.relayFlag = True
            print('Relaying Odometry to PX4')

    def vehicle_odometry_callback(self, msg):
        self.curPos = np.array([msg.position[0],-msg.position[1],-msg.position[2]])
        self.cur_vel = np.array([msg.velocity[0],-msg.velocity[1],-msg.velocity[2]])
        self.yaw = -euler_from_quaternion([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])[2]
        if self.relayFlag == False:
            self.odomFlag = True
            print('Odometry received')
        if self.curPos[2] > self.takeoffThreshold and not self.trajFlag:
            self.trajFlag = True



    def sp_callback(self, msg):
        self.posSp[0] = msg.pose.position.x
        self.posSp[1] = msg.pose.positionen.y
        quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.yawSp = euler_from_quaternion(quat)[2]

    def set_offboard(self):
        pass


    def acc2quat(self,des_a, des_yaw):
        xb_des = np.array([1, 0, 0.0])
        if np.linalg.norm(des_a) == 0.0:
            zb_des = np.array([0,0,1])
        else:    
            zb_des = des_a / np.linalg.norm(des_a)
        yb_des = np.cross(zb_des, xb_des) / np.linalg.norm(np.cross(zb_des, xb_des))
        proj_xb_des = np.cross(yb_des, zb_des) / np.linalg.norm(np.cross(yb_des, zb_des))
       
        rotmat = np.transpose(np.array([proj_xb_des, yb_des, zb_des]))
        return rotmat



    def cmdloop_callback(self):
        if self.odomFlag and self.relayFlag and self.trajFlag:
            norm_distance = np.linalg.norm(self.posSp - self.curPos)
            posSp_ = np.array([1.0,1.0,1.0])
            maxNorm_ = 0.3
            if norm_distance > maxNorm_:
                posSp_ = self.curPos + maxNorm_*(self.posSp - self.curPos)/norm_distance
            else:
                posSp_[0] = self.posSp[0]
                posSp_[1] = self.posSp[1]
                posSp_[2] = self.posSp[2]

            yawDiff_  = self.yawSp - self.yaw
            yawSum_ = self.yawSp + self.yaw
            if np.absolute(yawDiff_) > np.pi:
                yawDiff_ = -np.sign(yawDiff_)*(2*np.pi - yawDiff_)
            
            yawDiff_ = np.maximum(-0.1, np.minimum(0.1, yawDiff_))
            yawSp_ = self.yaw + yawDiff_

            self.quatSp = quaternion_from_euler(0.0, 0.0, yawSp_)

            now = self.node.get_clock().now().to_msg()

            self.posSpMsg.header.stamp = now
            self.posSpMsg.pose.position.x = posSp_[0]
            self.posSpMsg.pose.position.y = posSp_[1]
            self.posSpMsg.pose.position.z = posSp_[2]

            self.posSpMsg.pose.orientation.x = self.quatSp[0]
            self.posSpMsg.pose.orientation.y = self.quatSp[1]
            self.posSpMsg.pose.orientation.z = self.quatSp[2]
            self.posSpMsg.pose.orientation.w = self.quatSp[3]



            self.posSpPub.publish(self.posSpMsg)

        else:
            # print("Odometry not received. Please check the topics")
            pass



        

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    print("True")

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
