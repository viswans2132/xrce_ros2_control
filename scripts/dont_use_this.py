#!/usr/bin/env python

__author__ = "Viswa Narayanan Sankaranarayanan"
__contact__ = "vissan@ltu.se"

import rclpy
import numpy as np
from tf_transformations import *
import time
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry, VehicleAttitudeSetpoint, VehicleControlMode, VehicleLocalPosition

class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)

        # Subscribers
        self.global_position = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_position_callback, qos_profile)
        self.odometry = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.setpoint_position = self.create_subscription(VehicleLocalPosition, '/new_pose', self.sp_position_callback, qos_profile)
        self.mode_sub = self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, qos_profile)

        #Publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.publisher_attitude = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        self.publisher_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Constants and cutoff values
        self.gravity = np.array([0, 0, -9.81])
        self.max_acc = 5
        self.max_throttle = 0.7
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)


        # initial values for setpoints
        self.cur_pose = np.zeros((3,))
        self.cur_vel = np.zeros((3,))
        self.yaw = 0.0
        self.start_yaw = 1.0

        # Setpoints
        self.pos_sp = np.array([-0.0,-0.0,-0.5])
        self.vel_sp = np.array([0.0,0.0,0.0])
        self.yaw_sp = 1.57
        self.home_pos = np.array([0,0,-0.05])

        # Storage Variables
        self.desVel = np.zeros((3,))
        self.errInt = np.zeros((3,))
        self.errVel = np.zeros((3,))
        self.pre_time = Clock().now().nanoseconds/1E9
        self.offboard_time = Clock().now().nanoseconds/1E9
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.dt = timer_period

        # Gains
        self.Kpos = np.array([-1.0, -1.0, -1.3])
        self.Kvel = np.array([-0.8, -0.8, -1])
        self.Kder = np.array([-0.0, -0.0, -0.0])
        self.Kint = np.array([-0.0, -0.0, -0.0])
        self.norm_thrust_const = 0.17

        # Msg Variables
        self.att_cmd = VehicleAttitudeSetpoint()
        # self.data_out = PlotDataMsg()
        self.vehicle_ts = int(Clock().now().nanoseconds/1E3)


        # Flags
        self.armed = 1
        self.offboard_mode = False
        self.arm_flag = False
        self.mission_complete = False
        self.home = False

        print("Sleeping")

        time.sleep(2)
        print("Awake")
        self.mode()

    def vehicle_control_mode_callback(self, msg):
        self.offboard_mode = msg.flag_control_offboard_enabled
 
    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.armed = msg.arming_state

    def vehicle_position_callback(self, msg):
        self.cur_pose = np.array([msg.x,msg.y,msg.z])
        self.cur_vel = np.array([msg.vx,msg.vy,msg.vz])

    def vehicle_odometry_callback(self, msg):
        self.yaw = euler_from_quaternion([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])[2]
        self.vehicle_ts = msg.timestamp

    def sp_position_callback(self, msg):
        self.pos_sp = np.array([msg.x,msg.y,msg.z])

    def arm_uav(self):
        # Set to arm
        command_msg_arm = VehicleCommand()
        command_msg_arm.timestamp = int(Clock().now().nanoseconds / 1000)
        command_msg_arm.param1 = 1.0
        command_msg_arm.param2 = 0.0
        command_msg_arm.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        command_msg_arm.target_component = 1
        command_msg_arm.target_system = 1
        command_msg_arm.source_component = 1
        command_msg_arm.source_system = 1
        command_msg_arm.from_external = True
        self.publisher_command.publish(command_msg_arm) 

        # Set to offboard mode
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
        # print("Setting Arm")
        self.publisher_command.publish(command_msg_mode)

    def set_offboard(self):
        pass

    def disarm(self):
        # Set to arm
        command_msg_arm = VehicleCommand()
        command_msg_arm.timestamp = int(Clock().now().nanoseconds / 1000)
        command_msg_arm.param1 = 0.0
        command_msg_arm.param2 = 0.0
        command_msg_arm.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        command_msg_arm.target_component = 1
        command_msg_arm.target_system = 1
        command_msg_arm.source_component = 1
        command_msg_arm.source_system = 1
        command_msg_arm.from_external = True

    def mode(self):
        # Set offboard mode to position control
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=True
        offboard_msg.acceleration=True
        offboard_msg.attitude=True
        offboard_msg.body_rate=False
        # offboard_msg.actuator=False
        self.publisher_offboard_mode.publish(offboard_msg)


    def a_des(self):
        # dt = Clock().now().nanoseconds/1E9 - self.pre_time
        # self.pre_time = self.pre_time + dt
        # if dt > 0.04:
        #     dt = 0.04

        # self.time_from_start = Clock().now().nanoseconds/1E9 - self.offboard_time
        # self.pos_sp[0] = 2*np.cos(0.2*self.time_from_start)
        # self.pos_sp[1] = 2*np.sin(0.2*self.time_from_start)

        R = np.array([[np.cos(self.yaw), np.sin(self.yaw), 0], [-np.sin(self.yaw), np.cos(self.yaw), 0], [0, 0, 1]])
        cur_vel_W = self.cur_vel

        errPos = self.cur_pose - self.pos_sp
        desVel = self.Kpos * errPos

        derVel = ((self.cur_vel - desVel) - self.errVel)/self.dt;
        self.errVel = self.cur_vel - desVel;
        self.errInt = self.errInt + self.errVel*self.dt

        max_int = np.array([2, 2, 6])
        self.errInt = np.maximum(-max_int, np.minimum(max_int, self.errInt))

        des_a = np.zeros((3,))
        print([errPos,self.errVel[0]])


        des_a[0] = self.Kvel[0]*self.errVel[0] + self.Kder[0]*derVel[0] + self.Kint[0]*self.errInt[0]
        des_a[1] = self.Kvel[1]*self.errVel[1] + self.Kder[1]*derVel[1] + self.Kint[1]*self.errInt[1]
        des_a[2] = self.Kvel[2]*self.errVel[2] + self.Kder[2]*derVel[2] + self.Kint[2]*self.errInt[2]

        dA = np.zeros((3,))

        dA = R.dot(des_a)

        # Lines to copy
        max_des = np.array([0.2, 0.2, 5])
        dA = np.maximum(-max_des,(np.minimum(max_des, dA)))

        if np.linalg.norm(dA) > self.max_acc:
            dA = (self.max_acc/np.linalg.norm(dA))*dA

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
        self.mode()
        if(self.armed == 2 and self.offboard_mode == True):
            self.arm_flag = True
            # self.cnt = self.cnt + 1

            des_a = self.a_des()

            yaw_rad = -1.57

            yaw_diff = yaw_rad - self.yaw
            yaw_diff = np.maximum(-0.1, np.minimum(0.1, yaw_diff))


            yaw_ref = self.yaw + yaw_diff



            r_des = self.acc2quat(des_a, 0.0)

           
            zb = r_des[:,2]
            thrust = self.norm_thrust_const * des_a.dot(zb) + 1
            quat_des = quaternion_from_euler(des_a[1], -des_a[0], yaw_ref)
            thrust = np.maximum(-0.8, np.minimum(thrust, 0.8))
            print(yaw_ref, quat_des[2])


            now = int(Clock().now().nanoseconds/1E3)

            self.att_cmd.timestamp = self.vehicle_ts
            self.att_cmd.q_d[0] = quat_des[3]
            self.att_cmd.q_d[1] = quat_des[0]
            self.att_cmd.q_d[2] = quat_des[1]
            self.att_cmd.q_d[3] = quat_des[2]

            # self.att_cmd.q_d[0] = 1.0
            # self.att_cmd.q_d[1] = 0.0
            # self.att_cmd.q_d[2] = 0.0
            # self.att_cmd.q_d[3] = 0.0

            self.yaw_sp_move_rate = 0.0
            # print(thrust)
            # thrust = 0.0
            self.att_cmd.thrust_body[2] = thrust
            print(thrust)

            self.publisher_attitude.publish(self.att_cmd)
            

        elif self.arm_flag==False:
            self.arm_uav()
            print("Not armed yet")
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
