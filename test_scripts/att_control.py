#!/usr/bin/env python

__author__ = "Viswa Narayanan Sankaranrayanan"
__contact__ = "vissan@ltu.se"

HW_TEST = False # Make this true before using hardware
EXT_ODOM_SOURCE = "REALSENSE" # Make this "REALSENSE", while using realsense topics
EXT_ARMING = False # Make this true, if you want arming to be done from the remote control. Otherwise, this node will call an arming service.
AUTO_START = True # Make this true, if you want the controller to run without waiting for the takeoff signal.

import rclpy
import numpy as np
import time
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleAttitudeSetpoint
from sensor_msgs.msg import LaserScan, PointCloud2
from tf_transformations import quaternion_matrix, euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Header, String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class OffboardControl(Node):
    def __init__(self):
        super().__init__('att_control_lidar_2d_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_profile_2 = QoSProfile(depth=1)
        laser_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        qos_profile_3 = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1
        )

        self.declare_parameter('hw_test', HW_TEST)
        self.declare_parameter('ext_odom_source', EXT_ODOM_SOURCE)
        self.declare_parameter('ext_arming', EXT_ARMING)
        self.declare_parameter('auto_start', AUTO_START)

        self.hw_test = bool(self.get_parameter('hw_test').value)
        self.ext_odom_source = self.get_parameter('ext_odom_source').get_parameter_value().string_value
        self.ext_arming = bool(self.get_parameter('ext_arming').value)
        self.auto_start = bool(self.get_parameter('auto_start').value)


        self.safetyRadius = 1.0 # meters
        self.pos_sp = np.array([0.0, 380.0, 0.0])

        self.des_pos = np.array([0.0, 0.0, -600.8])
        self.errInt = np.array([0.0, 0.0, 0.0])

        self.lidar_rel_pos = np.array([0.12, 0.0, 0.26])

        timer_period = 0.01  # 50 Hz
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

    
        self.cur_pos = np.array([0, 0, 0])
        self.yaw = 0.0
        self.yaw_sp = 0.0
        self.cur_orien = np.zeros((4,1))
        self.dt = timer_period
        self.cnt = 4

        # Storage Variables
        self.desVel = np.zeros((3,))
        self.errInt = np.zeros((3,))
        self.errVel = np.zeros((3,))
        self.pre_time = Clock().now().nanoseconds/1E9
        self.offboard_time = Clock().now().nanoseconds/1E9
        self.landTime = Clock().now().nanoseconds/1E9
        self.time_from_start = 0.0

        self.armed = False
        self.offboard_mode = False
        self.arm_flag = False
        self.odomFlag = False
        self.takeOffFlag = False
        self.consFlag = False
        self.landFlag = False
        self.relayFlag = False
        self.controlFlag = False
        self.trajFlag = False
        self.waitFlag = False

        self.A = np.array([])
        self.b = np.array([])

        # Velocity clipping
        self.maxXVel = 0.5
        self.maxYVel = 0.5
        self.maxZVel = 0.3

        # Gains
        self.Kpos = np.array([-0.9, -0.9, -1.2])
        self.Kvel = np.array([-0.3, -0.3, -1.0])
        self.Kder = np.array([-0.0, -0.0, -0.0])
        self.Kint = np.array([-0.0, -0.0, -0.4])
        self.norm_thrust_const = 0.15

        # Constants and cutoff values
        self.gravity = np.array([0, 0, -10.0])
        self.max_acc = 5
        self.max_throttle = 0.7

        # Msg Variables
        self.att_cmd = VehicleAttitudeSetpoint()


        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.mode_sub = self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, qos_profile)
        self.takeOff_sub = self.create_subscription(String, '/takeoff', self.start_callback, qos_profile_2)
        self.pos_sp_sub = self.create_subscription(PoseStamped, '/ref', self.sp_position_callback, laser_qos)

        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode,'/fmu/in/offboard_control_mode',qos_profile)
        self.publisher_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.publisher_attitude = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)

        print("Sleeping")
        time.sleep(2)
        print("Awake")


        self.ext_odom_time = time.time()

        if self.hw_test:
            self.relay_sub = self.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', self.relay_callback, qos_profile)

            if self.ext_odom_source == "REALSENSE":
                self.ext_odom_sub = self.create_subscription(Odometry, '/ov_msckf/odomimu', self.ext_odom_callback, qos_profile_3)
                self.ext_timer = self.create_timer(0.1, self.ext_odom_check)
        else:
            self.relayFlag = True

        if self.auto_start:
            self.controlFlag = True

        self.mode()

    def sp_position_callback(self, msg):
        self.pos_sp[0] = msg.pose.position.x
        self.pos_sp[1] = -msg.pose.position.y
        self.pos_sp[2] = -msg.pose.position.z
        quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.yaw_sp = -euler_from_quaternion(quat)[2]
        print("New setpoint received")
        print(self.pos_sp)

    def vehicle_control_mode_callback(self, msg):
        self.offboard_mode = msg.flag_control_offboard_enabled
        self.armed = msg.flag_armed
        # print(['offboard:', msg.flag_control_offboard_enabled])


    def vehicle_odometry_callback(self, msg):
        if self.odomFlag == False:
            print("Odom")
            # self.pos_sp[0] = msg.position[0]
            # self.pos_sp[1] = msg.position[1]
            self.odomFlag = True
        self.cur_pos = np.array([msg.position[0], msg.position[1], msg.position[2]])
        self.cur_vel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])
        self.cur_orien = np.array([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])
        self.yaw = euler_from_quaternion([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])[2]
        if self.cur_pos[2] < -1000.0:
            if not self.takeOffFlag:
                print("Takeoff detected")                
            self.takeOffFlag = True
            self.des_pos[0] = self.pos_sp[0]
            self.des_pos[1] = self.pos_sp[1]

    def relay_callback(self, msg):
        if not self.relayFlag:
            self.relayFlag = True

    def start_callback(self, msg):
        if not self.controlFlag:
            self.controlFlag = True

    def land_callback(self, msg):
        self.landFlag = True

    def ext_odom_callback(self, msg):
        self.ext_odom_time = time.time()

    def ext_odom_check(self):
        time_diff = time.time() - self.ext_odom_time

        if time_diff > 0.4:
            self.landFlag = True
            print("External Odometry not received")

    def set_land(self):
        # Set to arm
        command_msg_land = VehicleCommand()
        command_msg_land.timestamp = int(Clock().now().nanoseconds / 1000)
        command_msg_land.param1 = 0.0
        command_msg_land.param2 = 0.0
        command_msg_land.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        command_msg_land.target_component = 1
        command_msg_land.target_system = 1
        command_msg_land.source_component = 1
        command_msg_land.source_system = 1
        command_msg_land.from_external = True
        self.publisher_command.publish(command_msg_land)

    def arm_offboard(self):
        # Arm
        command_msg_arm = VehicleCommand()
        command_msg_arm.timestamp = int(Clock().now().nanoseconds / 1000)
        command_msg_arm.param1 = 1.0
        command_msg_arm.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        command_msg_arm.target_component = 1
        command_msg_arm.target_system = 1
        command_msg_arm.source_component = 1
        command_msg_arm.source_system = 1
        command_msg_arm.from_external = True
        self.publisher_command.publish(command_msg_arm)

        # if self.armed == 2:
        # Offboard mode
        command_msg_mode = VehicleCommand()
        command_msg_mode.timestamp = int(Clock().now().nanoseconds / 1000)
        command_msg_mode.param1 = 1.0
        command_msg_mode.param2 = 6.0
        command_msg_mode.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        command_msg_mode.target_component = 1
        command_msg_mode.target_system = 1
        command_msg_mode.source_component = 1
        command_msg_mode.source_system = 1
        command_msg_mode.from_external = True
        self.publisher_command.publish(command_msg_mode)

    def mode(self):
        # Switch to velocity control
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = True
        offboard_msg.body_rate = False
        self.publisher_offboard_mode.publish(offboard_msg)

    def a_des(self):
        R = np.array([[np.cos(self.yaw), np.sin(self.yaw), 0], [-np.sin(self.yaw), np.cos(self.yaw), 0], [0, 0, 1]])
        cur_vel_W = R.T.dot(self.cur_vel)

        errPos = self.cur_pos - self.des_pos
        # print(errPos[2])

        if np.linalg.norm(errPos[2]) < 0.05:
            self.trajFlag = True

        if self.trajFlag == True:
            self.time_from_start = Clock().now().nanoseconds/1E9 - self.offboard_time
            # self.pos_sp[0] = 0.5*np.cos(0.4*self.time_from_start)
            # self.pos_sp[1] = 0.5*np.sin(0.4*self.time_from_start)


        # errPos[:2] = np.maximum(np.array([0.1, 0.1]), np.minimum(np.array([-0.1, -0.1]), errPos[:2]))
        # print(self.des_pos)
        desVel = self.Kpos * errPos
       


        derVel = ((self.cur_vel - desVel) - self.errVel)/self.dt
        self.errVel = self.cur_vel - desVel
        self.errInt = self.errInt + self.errVel*self.dt

        max_int = np.array([0, 0, 6])
        self.errInt = np.maximum(-max_int, np.minimum(max_int, self.errInt))

        des_a = np.zeros((3,))
        des_a[0] = self.Kvel[0]*self.errVel[0] + self.Kder[0]*derVel[0] + self.Kint[0]*self.errInt[0]
        des_a[1] = self.Kvel[1]*self.errVel[1] + self.Kder[1]*derVel[1] + self.Kint[1]*self.errInt[1]
        des_a[2] = self.Kvel[2]*self.errVel[2] + self.Kder[2]*derVel[2] + self.Kint[2]*self.errInt[2]

        dA = np.zeros((3,))
        dA = R.dot(des_a)

        max_des = np.array([0.1, 0.1, 5])
        min_des = np.array([-0.1, -0.1, -15])
        dA = np.maximum(min_des,(np.minimum(max_des, dA)))


        # if np.linalg.norm(dA) > self.max_acc:
        #     dA = (self.max_acc/np.linalg.norm(dA))*dA

        return (dA + self.gravity) 


    def acc2quat(self,des_a, des_yaw):
        xb_des = np.array([1, 0, 0.0])
        if np.linalg.norm(des_a) == 0.0:
            zb_des = np.array([0,0,1])
        else:    
            zb_des = -des_a / np.linalg.norm(des_a)
        yb_des = np.cross(zb_des, xb_des) / np.linalg.norm(np.cross(zb_des, xb_des))
        proj_xb_des = np.cross(yb_des, zb_des) / np.linalg.norm(np.cross(yb_des, zb_des))
       
        rotmat = np.transpose(np.array([proj_xb_des, yb_des, zb_des]))
        return rotmat



    def cmdloop_callback(self):
        alts = [600.0, 600.0, 1000.0, 1000.0, 200, 70, 20, 10]


        if self.landFlag:
            self.set_land()
        else:
            if self.odomFlag and self.relayFlag and self.controlFlag:
                self.mode()
                if(self.armed and self.offboard_mode):
                    self.arm_flag = True

                    des_a = self.a_des()

                    yaw_rad = self.yaw_sp
                    yaw_diff = yaw_rad - self.yaw
                    yaw_diff = np.maximum(-0.05, np.minimum(0.05, yaw_diff))

                    yaw_ref = self.yaw + yaw_diff

                    r_des = self.acc2quat(des_a, yaw_ref)

                    zb = r_des[:,2]
                    quat_des = quaternion_from_euler(des_a[1], -des_a[0], yaw_ref)

                    thrust = self.norm_thrust_const * des_a.dot(zb) + 1
                    thrust = np.maximum(-0.75, np.minimum(thrust, 0.75))
                    # print(yaw_ref, quat_des[3])

                    now = int(Clock().now().nanoseconds/1E3)

                    self.att_cmd.timestamp = now
                    self.att_cmd.q_d[0] = quat_des[3]
                    self.att_cmd.q_d[1] = quat_des[0]
                    self.att_cmd.q_d[2] = quat_des[1]
                    self.att_cmd.q_d[3] = quat_des[2]
                    self.att_cmd.thrust_body[2] = thrust

                    if self.cnt == 4:
                        self.att_cmd.q_d[0] = 1
                        self.att_cmd.q_d[1] = 0
                        self.att_cmd.q_d[2] = 0
                        self.att_cmd.q_d[3] = 0
                        self.att_cmd.thrust_body[2] = 0.9

                    
                    # print("thrust: {:.2f}".format(thrust))
                    # print(f"value: {des_a[0]:.3f}: {des_a[1]:.3f}:  {des_a[2]:.3f}")
                    if self.cnt < 1:
                        err = np.abs(self.des_pos[2] - self.cur_pos[2])
                    elif self.cnt == 4:
                        err = self.des_pos[2] - self.cur_pos[2]
                    else:
                        err = np.linalg.norm(self.des_pos - self.cur_pos)

                    if err < 0.1:
                        if not self.waitFlag:
                            self.offboard_time = Clock().now().nanoseconds/1E9
                            self.waitFlag = True
                            print("Offboard time reset")

                        self.time_from_start = Clock().now().nanoseconds/1E9 - self.offboard_time
                        # print(self.time_from_start)
                        if self.cnt >= 4 and self.cnt < 6:
                            wait_time = 1.0
                        else:
                            wait_time = 5.0


                        if self.waitFlag:
                            if self.time_from_start > wait_time:
                                self.waitFlag = False
                                self.cnt = self.cnt+1
                                if self.cnt == len(alts):
                                    self.landFlag = True
                                    self.set_land()
                                else:
                                    self.des_pos[2] = -alts[self.cnt]
                                    print("Wait over: Updating next point")


                        else:
                            if self.cnt == len(alts):
                                if np.linalg.norm(self.des_pos - self.cur_pos) < 0.1:
                                    self.landFlag = True
                                    self.set_land()
                                    print("Landing")
                            else:
                                print("****")
                                print("****")
                                print("****")
                                print(f"Setpoint reached: {alts[self.cnt]:.1f} m")
                                print("****")
                                print("****")
                                print("****")
                                self.des_pos[2] = -alts[self.cnt]
                                self.cnt = self.cnt + 1
                                print("Updating next point")
                    if self.cnt == 1:
                        self.des_pos[0] = -30.0
                        self.des_pos[1] = 200.0
                    if self.cnt == 2:
                        self.des_pos[0] = -30.0
                        self.des_pos[1] = 200.0
                    if self.cnt == 3:
                        self.des_pos[0] = 0.0
                        self.des_pos[1] = 450.0
                    if self.cnt > 3:
                        self.des_pos[0] = 0.0
                        self.des_pos[1] = 380.0

                    print(f"Current Position: {self.cur_pos[1]:.1f}, {-self.cur_pos[0]:.1f}, {-self.cur_pos[2]:.1f} m") 

                    self.publisher_attitude.publish(self.att_cmd)

                elif not self.arm_flag:
                    self.arm_offboard()
            else:
                print(f"Odometry Flag: {self.odomFlag}, FMU Relay Flag: {self.relayFlag}, Control Flag: {self.controlFlag}")


def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
