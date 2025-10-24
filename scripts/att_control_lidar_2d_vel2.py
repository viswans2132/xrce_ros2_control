#!/usr/bin/env python

__author__ = "Viswa Narayanan Sankaranrayanan"
__contact__ = "vissan@ltu.se"

HW_TEST = True # Make this true before using hardware
EXT_ODOM_SOURCE = "REALSENSE" # Make this "REALSENSE", while using realsense topics
EXT_ARMING = False # Make this true, if you want arming to be done from the remote control. Otherwise, this node will call an arming service.

import rclpy
import numpy as np
import cvxpy as cp
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
import ros2_numpy
from tf_transformations import quaternion_matrix, euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Header, String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

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


        self.safetyRadius = 1.0 # meters
        self.vel_sp = np.array([0.0, 0.0, 0.0])

        self.des_vel = np.array([0.0, 0.0, 0.0])
        self.errInt = np.array([0.0, 0.0, 0.0])

        self.lidar_rel_pos = np.array([0.12, 0.0, 0.26])

        timer_period = 0.01  # 50 Hz

    
        self.cur_pos = np.array([0, 0, 0])
        self.yaw = 0.0
        self.yaw_sp = 0.0
        self.cur_orien = np.zeros((4,1))
        self.dt = timer_period
        self.cnt = 0

        # Storage Variables
        self.desVel = np.zeros((3,))
        self.errInt = np.zeros((3,))
        self.errVel = np.zeros((3,))
        self.pre_time = Clock().now().nanoseconds/1E9
        self.offboard_time = Clock().now().nanoseconds/1E9
        self.landTime = Clock().now().nanoseconds/1E9

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

        self.A = np.array([])
        self.b = np.array([])

        # Velocity clipping
        self.maxXVel = 0.5
        self.maxYVel = 0.5
        self.maxZVel = 0.3

        # Gains
        self.Kpos = np.array([-0.6, -0.6, -1.2])
        self.Kvel = np.array([-0.5, -0.5, -1.0])
        self.Kder = np.array([-0.0, -0.0, -0.0])
        self.Kint = np.array([-0.1, -0.1, -0.4])
        self.norm_thrust_const = 0.155

        # Constants and cutoff values
        self.gravity = np.array([0, 0, -10.0])
        self.max_acc = 5
        self.max_throttle = 0.7

        # Msg Variables
        self.att_cmd = VehicleAttitudeSetpoint()


        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.mode_sub = self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, qos_profile)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.laserscan_callback, laser_qos)
        self.takeOff_sub = self.create_subscription(String, '/takeoff', self.start_callback, qos_profile_2)
        self.vel_sp_sub = self.create_subscription(Twist, '/cmd_vel', self.sp_velocity_callback, laser_qos)

        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode,'/fmu/in/offboard_control_mode',qos_profile)
        self.publisher_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.publisher_attitude = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        self.publisher_points = self.create_publisher(PointCloud2, '/reduced_points', laser_qos)

        print("Sleeping")
        time.sleep(2)
        print("Awake")

        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.ext_odom_time = time.time()

        if HW_TEST:
            self.relay_sub = self.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', self.relay_callback, qos_profile)

            if EXT_ODOM_SOURCE == "REALSENSE":
                self.ext_odom_sub = self.create_subscription(Odometry, '/ov_msckf/odomimu', self.ext_odom_callback, qos_profile_3)
                self.ext_timer = self.create_timer(0.1, self.ext_odom_check)

        self.mode()

    def sp_velocity_callback(self, msg):
        self.vel_sp[0] = msg.linear.x
        self.vel_sp[1] = -msg.linear.y
        self.vel_sp[2] = -msg.linear.z
        # quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        # self.yaw_sp = -euler_from_quaternion(quat)[2]
        print("New setpoint received")
        print(self.vel_sp)

    def vehicle_control_mode_callback(self, msg):
        self.offboard_mode = msg.flag_control_offboard_enabled
        self.armed = msg.flag_armed
        # print(['offboard:', msg.flag_control_offboard_enabled])


    def vehicle_odometry_callback(self, msg):
        self.cur_pos = np.array([msg.position[0], msg.position[1], msg.position[2]])
        self.cur_vel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])
        self.cur_orien = np.array([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])
        self.yaw = euler_from_quaternion([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])[2]
        if self.cur_pos[2] < 0.45:
            if not self.takeOffFlag:
                print("Takeoff detected")                
            self.takeOffFlag = True
            self.des_vel[0] = self.vel_sp[0]
            self.des_vel[1] = self.vel_sp[1]

        if self.odomFlag == False:
            self.odomFlag = True
            self.yaw_sp = self.yaw

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

        if time_diff > 0.2:
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

    def laserscan_callback(self, msg: LaserScan):
        # Step 1: Convert polar scan to (x,y) in LiDAR frame
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges)

        # Filter out invalid measurements
        mask = np.isfinite(ranges) & (ranges > 0.40) & (ranges < 2.75)
        ranges = ranges[mask]
        angles = angles[mask]

        # Cartesian (x, y, z=0) in LiDAR frame
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)

        points = np.vstack((x, y, z)).T   # Nx3 array

        # Step 2: Voxelization (same as before)
        voxel_size = 1.0
        discrete_coords = np.floor(points / voxel_size).astype(np.int32)
        _, unique_indices = np.unique(discrete_coords, axis=0, return_index=True)
        points = points[unique_indices] 
        self.points_array = np.array([points[:,0], -points[:,1], -points[:,2]]).T



        trans_points = points  + self.lidar_rel_pos
        rot = quaternion_matrix(self.cur_orien)[:-1, :-1]

        rot_points = trans_points@rot + np.array([self.cur_pos[0], -self.cur_pos[1], -self.cur_pos[2]])

        # Step 4: Flag and generate constraints
        if len(self.points_array) < 1:
            self.consFlag = False
        else:
            self.genConsMatrix()

        self.republish_points(rot_points)

    def republish_points(self, rot_points):
        points_xyz = np.zeros((len(rot_points),), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('r', np.uint8),
            ('g', np.uint8),
            ('b', np.uint8)])


        points_xyz['x'] = rot_points[:,0]
        points_xyz['y'] = rot_points[:,1]
        points_xyz['z'] = rot_points[:,2]
        points_xyz['r'] = 255
        points_xyz['g'] = 255
        points_xyz['b'] = 255

        pcl = ros2_numpy.msgify(PointCloud2, points_xyz)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()  # current ROS 2 timestamp
        header.frame_id = 'map'                # must exist in TF tree
        pcl.header = header

        self.publisher_points.publish(pcl)


    def genConsMatrix(self):
        # Rotation from quaternion
        R = quaternion_matrix(self.cur_orien)[:-1, :-1]

        rotated_points = (R @ self.points_array.T).T
        points_xy = rotated_points[:, :2]


        if len(rotated_points) < 1:
            self.consFlag = False
        else:
            self.A = -2 * points_xy
            dist = np.sqrt(np.sum(points_xy**2, axis=1)) - self.safetyRadius
            self.b = -1.5 * (np.sum(points_xy**2, axis=1) - self.safetyRadius**2)
            # print(dist)

            if self.takeOffFlag and not self.consFlag:
                self.consFlag = True
                print('Safety ON')


    def safety_filter(self, desVel):
        # print("Safety ON")
        P = np.eye(2)
        u = cp.Variable(2)

        print(f"Before: {desVel[0]:.2f}, {desVel[1]:.2f}")

        if len(self.A) == len(self.b):
            constraints = [self.A@u >= self.b]
            prob = cp.Problem(cp.Minimize(cp.quad_form(u-desVel, P)), constraints)

            try:
                result = prob.solve()
                val = u.value

                try:
                    val = np.maximum(-np.array([self.maxXVel, self.maxYVel]), np.minimum(np.array([self.maxXVel, self.maxYVel]), val))
                except TypeError:
                    print("Safety filter returned None")
                    val = np.array([0.0, 0.0])

            except cp.error.SolverError:
                print("Safety filter: solver error")
                val =  np.array([0.0, 0.0])

            print(f"After: {val[0]:.2f}, {val[1]:.2f}")

            return val
        
        else:
            return np.zeros(desVel.shape)



    def a_des(self):
        R = np.array([[np.cos(self.yaw), np.sin(self.yaw), 0], [-np.sin(self.yaw), np.cos(self.yaw), 0], [0, 0, 1]])
        cur_vel_W = R.T.dot(self.cur_vel)

        maxVel = np.array([self.maxXVel, self.maxYVel, self.maxZVel])
        self.des_vel = self.vel_sp.copy()
        desVel = np.clip(self.des_vel, -maxVel, maxVel)

        if self.consFlag:
            desVel[:2] = self.safety_filter(desVel[:2])
        

        derVel = ((self.cur_vel - desVel) - self.errVel)/self.dt
        self.errVel = self.cur_vel - desVel
        self.errInt = self.errInt + self.errVel*self.dt

        max_int = np.array([2, 2, 6])
        self.errInt = np.maximum(-max_int, np.minimum(max_int, self.errInt))

        des_a = np.zeros((3,))
        des_a[0] = self.Kvel[0]*self.errVel[0] + self.Kder[0]*derVel[0] + self.Kint[0]*self.errInt[0]
        des_a[1] = self.Kvel[1]*self.errVel[1] + self.Kder[1]*derVel[1] + self.Kint[1]*self.errInt[1]
        des_a[2] = self.Kvel[2]*self.errVel[2] + self.Kder[2]*derVel[2] + self.Kint[2]*self.errInt[2]

        dA = np.zeros((3,))
        dA = R.dot(des_a)

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

                    r_des = self.acc2quat(des_a, 0.0)

                    zb = r_des[:,2]
                    quat_des = quaternion_from_euler(des_a[1], -des_a[0], yaw_ref)

                    thrust = self.norm_thrust_const * des_a.dot(zb) + 1
                    thrust = np.maximum(-0.9, np.minimum(thrust, 0.9))
                    # print(yaw_ref, quat_des[3])

                    now = int(Clock().now().nanoseconds/1E3)

                    self.att_cmd.timestamp = now
                    self.att_cmd.q_d[0] = quat_des[3]
                    self.att_cmd.q_d[1] = quat_des[0]
                    self.att_cmd.q_d[2] = quat_des[1]
                    self.att_cmd.q_d[3] = quat_des[2]
                    
                    self.att_cmd.thrust_body[2] = thrust
                    # print("thrust: {:.3f}".format(thrust))
                    # print(f"value: {des_a[0]:.3f}: {des_a[1]:.3f}:  {des_a[2]:.3f}")

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
