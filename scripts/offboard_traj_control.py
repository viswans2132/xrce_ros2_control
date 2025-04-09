#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"

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
from px4_msgs.msg import VehicleLocalPosition

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

        self.global_position = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_position_callback, qos_profile)
        self.mode_sub = self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, qos_profile)

        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.publisher_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

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


        print("Sleeping")

        time.sleep(2)
        print("Awake")
        self.mode()
 
 
    def vehicle_control_mode_callback(self, msg):
        # print(msg.flag_control_offboard_enabled)
        self.offboard_mode = msg.flag_control_offboard_enabled
        print(['offboard:',msg.flag_control_offboard_enabled])
 
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        #print("NAV_STATUS: ", msg.nav_state)
        #print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
        self.armed = msg.arming_state
        print(['armed:',msg.arming_state])

    def vehicle_position_callback(self, msg):
        self.pos = np.array([msg.x,msg.y,msg.z])

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

        if self.armed==2:

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
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        offboard_msg.attitude=False
        offboard_msg.body_rate=False
        # offboard_msg.actuator=False
        self.publisher_offboard_mode.publish(offboard_msg)

    def cmdloop_callback(self):
        self.mode()

        if(self.armed == 2 and self.offboard_mode == True):
            self.arm_flag = True

            #Publish trajectory setpoint
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            trajectory_msg.position[0] = 0.0 #self.radius * np.cos(self.theta)
            trajectory_msg.position[1] = 0.0 #self.radius * np.sin(self.theta)
            trajectory_msg.position[2] = -1.2 #self.h
            # trajectory_msg.velocity[0] = float("nan")
            # trajectory_msg.velocity[1] = float("nan")
            # trajectory_msg.velocity[2] = float("nan")
            self.publisher_trajectory.publish(trajectory_msg)

            # self.h = self.h + self.vz * self.dt
            # self.theta = self.theta + self.omega * self.dt
            self.cnt = self.cnt + 1
        elif(self.arm_flag==False):
            self.arm_offboard()

def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
