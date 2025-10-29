#!/usr/bin/env python3
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

# from tf_transformations import quaternion_matrix

class OffboardControl(Node):

    def __init__(self):
        super().__init__('velocity_offboard_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_ALL,
            depth=1
        )

        self.safetyRadius = 1.0 # meters

        # self.status_sub = self.create_subscription(
        #     VehicleStatus,
        #     '/fmu/out/vehicle_status',
        #     self.vehicle_status_callback,
        #     qos_profile)

        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)

        self.mode_sub = self.create_subscription(
            VehicleControlMode,
            '/fmu/out/vehicle_control_mode',
            self.vehicle_control_mode_callback,
            qos_profile)

        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile)

        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile)

        self.publisher_command = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile)

        self.des_pos = np.array([0.0, 0.0, -1.0])
        self.errInt = np.array([0.0, 0.0, 0.0])

        timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.pos = np.array([0, 0, 0])
        self.dt = timer_period
        self.cnt = 0

        self.armed = False
        self.offboard_mode = False
        self.arm_flag = False

        self.odomFlag = False
        self.takeOffFlag = False
        self.cur_orien = np.array([0, 0, 0, 1])

        print("Sleeping")
        time.sleep(2)
        print("Awake")
        self.mode()

    def vehicle_control_mode_callback(self, msg):
        self.offboard_mode = msg.flag_control_offboard_enabled
        self.armed = msg.flag_armed
        # print(['offboard:', msg.flag_control_offboard_enabled])

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.armed = msg.arming_state
        print(['armed:', msg.arming_state])


    def vehicle_odometry_callback(self, msg):
        if self.odomFlag == False:
            self.des_pos[0] = msg.position[0]
            self.des_pos[1] = msg.position[1]
            self.odomFlag = True
        self.pos = np.array([msg.position[0],msg.position[1],msg.position[2]])
        # self.cur_vel = np.array([msg.velocity[0],msg.velocity[1],msg.velocity[2]])
        self.cur_orien = np.array([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])
        # self.yaw = euler_from_quaternion([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])[2]
        if self.pos[2] < -0.45:
            if not self.takeOffFlag:
                print("Takeoff detected")
            self.takeOffFlag = True

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
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.publisher_offboard_mode.publish(offboard_msg)




    def cmdloop_callback(self):
        if self.odomFlag:
            self.mode()
            print([self.armed ,self.offboard_mode])

            if (self.armed and self.offboard_mode):
                self.arm_flag = True

                if self.takeOffFlag:
                    v_cmd = np.array([0.0, 0.0, -0.5])

                else:
                    # print("Publishing")

                    error = self.des_pos - self.pos
                    # self.errInt = self.errInt + error*self.dt

                    vel_cmd = 0.5*error 

                    v_min = np.array([-0.2, -0.2, -1.0])
                    v_max = np.array([0.2, 0.2, 0.2])
                    v_cmd = np.maximum(v_min, np.minimum(v_max, vel_cmd))
                    print(f"Error: {error[0]:.2f}, {error[1]:.2f}, {error[2]:.2f}")

                # Publish velocity setpoint
                trajectory_msg = TrajectorySetpoint()
                trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)

                # Example: forward velocity of 1 m/s, downward velocity 0.2 m/s
                trajectory_msg.velocity[0] = v_cmd[0]   # vx [m/s]
                trajectory_msg.velocity[1] = v_cmd[1]   # vy [m/s]
                trajectory_msg.velocity[2] = v_cmd[2]  # vz [m/s]

                # Optional: yaw rate (set yaw to NaN if not used)
                trajectory_msg.yaw = float("nan")

                # Set positions to NaN (not controlled)
                trajectory_msg.position = [float("nan"), float("nan"), float("nan")]

                self.publisher_trajectory.publish(trajectory_msg)
                self.cnt += 1

            elif not self.arm_flag:
                self.arm_offboard()
        else:
            print("Odometry not received")


def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
