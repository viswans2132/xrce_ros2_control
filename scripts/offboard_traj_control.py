#!/usr/bin/env python

__author__ = "Viswa Narayanan Sankaranrayanan"
__contact__ = "vissan@ltu.se"

HW_TEST = False # Make this true before using hardware
EXT_ODOM_SOURCE = "VICON" # Make this "REALSENSE", while using realsense topics
EXT_ARMING = False # Make this true, if you want arming to be done from the remote control. Otherwise, this node will call an arming service.

import rclpy
import numpy as np
import time
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleOdometry

from std_msgs.msg import String, Header

# Not used in this file. They are here just for testing.
from tf_transformations import quaternion_matrix, euler_from_quaternion
import ros2_numpy
import cvxpy as cp


class OffboardControl(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_profile_2 = QoSProfile(depth=1)
        qos_profile_volatile_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.pos = [0,0,0]
        self.dt = timer_period
        self.theta = 0.0
        self.radius = 10.0
        self.omega = 0.1
        self.vz = -0.2
        self.h = -5
        self.cnt = 0

        self.length = 10
        self.height = 50
        self.res = 1
        self.bias = np.array([0,0,5])

        self.cur_pos = np.array([0.0, 0.0, 0.0])
        self.cur_ori = np.array([0.0, 0.0, 0.0, 1.0])
        self.pos_sp = np.array([0.0, 0.0, 0.0])
        self.odomFlag = False
        self.relayFlag = False
        self.controlFlag = False

        xmesh,zmesh = np.meshgrid(np.arange(self.bias[0],self.bias[0]+self.length,1/self.res),np.arange(self.bias[2],self.bias[2]+self.height,1/self.res))
        xmesh[1::2,:] = xmesh[1::2,::-1]
        self.num_wp = np.size(xmesh)
        x_wp = np.reshape(xmesh,self.num_wp)
        y_wp = np.zeros(self.num_wp) + self.bias[1]
        z_wp = -np.reshape(zmesh,self.num_wp)
        self.wp_list = np.stack((x_wp,y_wp,z_wp),axis=1)

        #self.wp_list = np.array([[-5.0, -5.0, -5.0], [-5.0, 5.0, -5.0], [5.0, 5.0, -5.0], [5.0, -5.0, -5.0]])
        self.iter = 0
        self.waypoint = self.wp_list[self.iter,:]

        self.track_acc = 0.25
        self.stay_time = 1

        self.wait_iter = 0
        self.wait_iter_max = int(self.stay_time/self.dt)

        
        self.armed = 1
        self.offboard_mode = False
        self.arm_flag = False
        self.mission_complete = False
        self.home = False
        self.home_pos = np.array([0,0,-0.05])


        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.mode_sub = self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, qos_profile)
        self.takeOff_sub = self.create_subscription(String, '/takeoff', self.start_callback)
        self.pos_sp_sub = self.create_subscription(Pose, '/ref', self.pos_sp_callback, qos_profile_volatile_reliable)

        if HW_TEST:
            self.relay_sub = self.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', self.relay_callback)

        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.publisher_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)


        print("Sleeping")

        time.sleep(2)
        print("Awake")
        self.mode()
 
 
    def pos_sp_callback(self, msg):
        # print(msg.flag_control_offboard_enabled)
        self.pos_sp[0] = msg.position.x
        self.pos_sp[1] = -msg.position.y
        self.pos_sp[2] = -msg.position.z

    def vehicle_control_mode_callback(self, msg):
        # print(msg.flag_control_offboard_enabled)
        self.offboard_mode = msg.flag_control_offboard_enabled
        self.armed = msg.flag_armed
        print(['offboard:',msg.flag_control_offboard_enabled])

    def vehicle_odometry_callback(self, msg):
        if self.odomFlag == False:
            self.pos_sp[0] = msg.position[0]
            self.pos_sp[1] = msg.position[1]
            self.odomFlag = True
        self.pos = np.array([msg.position[0],msg.position[1],msg.position[2]])
        # self.cur_vel = np.array([msg.velocity[0],msg.velocity[1],msg.velocity[2]])
        self.cur_ori = np.array([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])
        # self.yaw = euler_from_quaternion([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])[2]
        if self.pos[2] < -0.45:
            if not self.takeOffFlag:
                print("Takeoff detected")
            self.takeOffFlag = True

    def relay_callback(self, msg):
        if not self.relayFlag:
            self.relayFlag = True

    def start_callback(self, msg):
        if not self.controlFlag:
            self.controlFlag = True

    def arm_offboard(self):
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

        # if self.armed==2:

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

        # if self.cnt >= 10:
        self.publisher_command.publish(command_msg_mode) 

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
        offboard_msg.position=False
        offboard_msg.velocity=True
        offboard_msg.acceleration=False
        offboard_msg.attitude=False
        offboard_msg.body_rate=False
        # offboard_msg.actuator=False
        self.publisher_offboard_mode.publish(offboard_msg)

    def cmdloop_callback(self):
        if self.odomFlag and self.relayFlag and self.controlFlag:
            self.mode()

            if(self.armed == 2 and self.offboard_mode == True):
                self.arm_flag = True

                #Publish trajectory setpoint
                trajectory_msg = TrajectorySetpoint()
                trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
                trajectory_msg.position[0] = self.pos_sp[0] # + self.radius * np.cos(self.theta)
                trajectory_msg.position[1] = self.pos_sp[1] # + self.radius * np.sin(self.theta)
                trajectory_msg.position[2] = self.pos_sp[2] 
                # trajectory_msg.velocity[0] = float("nan")
                # trajectory_msg.velocity[1] = float("nan")
                # trajectory_msg.velocity[2] = float("nan")
                self.publisher_trajectory.publish(trajectory_msg)

                # self.h = self.h + self.vz * self.dt
                # self.theta = self.theta + self.omega * self.dt
                self.cnt = self.cnt + 1
            elif(self.arm_flag==False):
                self.arm_offboard()
        else:
            print(f"Odometry Flag: {self.odomFlag}, FMU Relay Flag: {self.relayFlag}, Takeoff Flag: {self.controlFlag}")

def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
