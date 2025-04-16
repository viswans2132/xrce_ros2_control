#!/usr/bin/env python

__author__ = "Viswa Narayanan Sankaranarayanan"
__contact__ = "vissan@ltu.se"

import rclpy
import numpy as np
import cvxpy as cp
from tf_transformations import *
import time
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, TwistStamped, Vector3
from nav_msgs.msg import Odometry
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry, VehicleAttitudeSetpoint, VehicleControlMode, VehicleLocalPosition
from std_msgs.msg import Header, Int8, String
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import ros2_numpy




class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_profile_be_vo = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_profile_re_vo = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_profile_volatile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)

        # Subscribers
        self.odometry = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.lio_odom = self.create_subscription(Odometry, '/shafterx2/odometry/imu', self.lio_callback, qos_profile_be_vo)
        self.posSpSub = self.create_subscription(PoseStamped, '/ref', self.sp_position_callback, qos_profile_volatile)
        self.mode_sub = self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, qos_profile)
        self.landSub = self.create_subscription(String, '/shafterx2/land', self.land_callback, qos_profile)
        self.takeOffSub = self.create_subscription(String, '/shafterx2/takeoff', self.start_callback, qos_profile)
        self.lidarSub = self.create_subscription(PointCloud2, '/shafterx2/ouster/points', self.pointcloud_callback, qos_profile_re_vo)

        #Publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.publisher_attitude = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        self.publisher_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.publisher_points = self.create_publisher(PointCloud2, '/reduced_points', qos_profile_re_vo)

        # Constants and cutoff values
        self.gravity = np.array([0, 0, -10.0])
        self.max_acc = 5
        self.max_throttle = 0.7
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)


        # initial values for setpoints
        self.cur_pose = np.zeros((3,1))
        self.cur_orien = np.zeros((4,1))
        self.cur_vel = np.zeros((3,1))
        self.R = np.eye(3)
        self.yaw = 0.0
        self.start_yaw = 1.0

        # Setpoints
        self.pos_sp = np.array([0.0, 0.0, -1.0])
        self.vel_sp = np.array([0.0, 0.0, 0.0])
        self.yaw_sp = 0.0
        self.home_pos = np.array([0, 0, 0.05])

        # Storage Variables
        self.desVel = np.zeros((3,))
        self.errInt = np.zeros((3,))
        self.errVel = np.zeros((3,))
        self.pre_time = Clock().now().nanoseconds/1E9
        self.offboard_time = Clock().now().nanoseconds/1E9
        self.landTime = Clock().now().nanoseconds/1E9
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.dt = timer_period

        self.points_array = np.array([])
        self.A = np.array([])
        self.b = np.array([])
        self.safetyRadius = 1.6
        self.consFlag = False

        # Gains
        self.Kpos = np.array([-0.6, -0.6, -1.2])
        self.Kvel = np.array([-0.5, -0.5, -1.0])
        self.Kder = np.array([-0.0, -0.0, -0.0])
        self.Kint = np.array([-0.1, -0.1, -0.4])
        self.norm_thrust_const = 0.155

        # Msg Variables
        self.att_cmd = VehicleAttitudeSetpoint()
        # self.data_out = PlotDataMsg()


        # Flags
        self.armed = 1
        self.odomFlag = False
        self.lioFlag = False
        self.offboard_mode = False
        self.position_mode = False
        self.arm_flag = False
        self.mission_complete = False
        self.home = False
        self.trajFlag = False
        self.landFlag = False
        self.takeOffFlag = False
        self.controlFlag = False


        print("Sleeping")

        time.sleep(2)
        print("Awake")
        self.mode()

    def vehicle_control_mode_callback(self, msg):
        self.offboard_mode = msg.flag_control_offboard_enabled
        self.position_mode = msg.flag_control_position_enabled
 
    def vehicle_status_callback(self, msg):
        if msg.arming_state == 2 and self.armed != 2:
            print(f'Armed and ready for offboard.')
            time.sleep(2)
        self.nav_state = msg.nav_state
        self.armed = msg.arming_state


    def vehicle_odometry_callback(self, msg):
        if self.odomFlag == False:
            self.pos_sp[0] = msg.position[0]
            self.pos_sp[1] = msg.position[1]
            self.odomFlag = True
        self.cur_pose = np.array([msg.position[0],msg.position[1],msg.position[2]])
        self.cur_vel = np.array([msg.velocity[0],msg.velocity[1],msg.velocity[2]])
        self.cur_orien = np.array([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])
        self.yaw = euler_from_quaternion([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])[2]
        if self.cur_pose[2] < -0.5:
            self.takeOffFlag = True
            print("Takeoff detected")

    def lio_callback(self, msg):
        if not self.lioFlag:
            print("Lio received")
        self.lioFlag = True

    def start_callback(self, msg):
        self.controlFlag = True
        self.landFlag = False

    def set_position(self):
        # Set to position control mode
        command_msg_mode = VehicleCommand()
        command_msg_mode.timestamp = int(Clock().now().nanoseconds / 1000)
        command_msg_mode.param1 = 1.0
        command_msg_mode.param2 = 3.0
        command_msg_mode.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        command_msg_mode.target_component = 1
        command_msg_mode.target_system = 1
        command_msg_mode.source_component = 1
        command_msg_mode.source_system = 1
        command_msg_mode.from_external = True
        # print("Setting Position Mode")
        self.publisher_command.publish(command_msg_mode)

    def land_callback(self, msg):
        self.landFlag = True
        print('Landing')

    def sp_position_callback(self, msg):
        self.pos_sp[0] = msg.pose.position.x
        self.pos_sp[1] = -msg.pose.position.y
        self.pos_sp[2] = -msg.pose.position.z
        quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.yaw_sp = -euler_from_quaternion(quat)[2]
        print("New setpoint received")
        print(self.pos_sp)

    def arm_uav(self):
        if self.position_mode == False and self.offboard_mode == False:
            self.set_position()
        else:
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
            print('Trying to arm')

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
                # print("Setting offboard")
                self.publisher_command.publish(command_msg_mode)


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
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        offboard_msg.attitude=True
        # offboard_msg.body_rate=False
        # offboard_msg.thrust_and_torque=False
        # offboard_msg.direct_actuator=False
        # offboard_msg.actuator=False
        self.publisher_offboard_mode.publish(offboard_msg)

    def pointcloud_callback(self, msg):
        pc_struct = ros2_numpy.numpify(msg)
        points = pc_struct['xyz']

        distances = np.linalg.norm(points, axis=1)
        points = points[(distances > 0.75) & (distances < 2.75)]

        voxel_size = 0.75
        discrete_coords = np.floor(points/voxel_size).astype(np.int32)
        _, unique_indices = np.unique(discrete_coords, axis=0, return_index=True)

        points = points[unique_indices]
        self.points_array = np.array([points[:,0], -points[:,1], -points[:,2]]).T        

        if len(self.points_array) < 1:
            self.consFlag = False
        else:
            self.genConsMatrix()

        pcl = PointCloud2()
        pcl = pc2.create_cloud_xyz32(msg.header, points.tolist())
        self.publisher_points.publish(pcl)


    def genConsMatrix(self):
        R = quaternion_matrix(self.cur_orien)[:-1, :-1]
        rotated_points = (R@self.points_array.T).T
        rotated_points = rotated_points[rotated_points[:,2] + self.cur_pose[2] < -0.01]

        if len(rotated_points) < 1:
            self.consFlag = False

        else:
            self.A = -2*rotated_points
            self.b = -1.5*(np.sum(rotated_points**2, 1) - self.safetyRadius**2)

            if self.takeOffFlag:
                self.consFlag = True


    def safety_filter(self, desVel):
        print("Safety ON")
        P = np.eye(3)
        u = cp.Variable(3)

        if len(self.A) == len(self.b):
            constraints = [self.A@u >= self.b]
            prob = cp.Problem(cp.Minimize(cp.quad_form(u-desVel, P)), constraints)

            try:
                result = prob.solve()
                val = u.value

                try:
                    val = np.maximum(-np.array([0.3, 0.3, 0.3]), np.minimum(np.array([0.3, 0.3, 0.3]), val))
                except TypeError:
                    print("Safety filter returned None")
                    val = np.array([0.0, 0.0, 0.0])

            except cp.error.SolverError:
                print("Safety filter: solver error")
                val =  np.array([0.0, 0.0, 0.0])

            return val
        
        else:
            return np.zeros(desVel.shape)



    def a_des(self):
        R = np.array([[np.cos(self.yaw), np.sin(self.yaw), 0], [-np.sin(self.yaw), np.cos(self.yaw), 0], [0, 0, 1]])
        cur_vel_W = R.T.dot(self.cur_vel)

        errPos = self.cur_pose - self.pos_sp

        if np.linalg.norm(errPos[2]) < 0.05:
            self.trajFlag = True

        if self.trajFlag == True:
            self.time_from_start = Clock().now().nanoseconds/1E9 - self.offboard_time
            # self.pos_sp[0] = 0.5*np.cos(0.4*self.time_from_start)
            # self.pos_sp[1] = 0.5*np.sin(0.4*self.time_from_start)


        desVel = self.Kpos * errPos

        if self.consFlag:
            desVel = self.safety_filter(desVel)
        
        # print(f"value: {desVel[0]:.3f}: {desVel[1]:.3f}:  {desVel[2]:.3f}")

        derVel = ((self.cur_vel - desVel) - self.errVel)/self.dt
        self.errVel = self.cur_vel - desVel
        self.errInt = self.errInt + self.errVel*self.dt

        max_int = np.array([2, 2, 6])
        self.errInt = np.maximum(-max_int, np.minimum(max_int, self.errInt))

        des_a = np.zeros((3,))


        des_a[0] = self.Kvel[0]*self.errVel[0] + self.Kder[0]*derVel[0] + self.Kint[0]*self.errInt[0]
        des_a[1] = self.Kvel[1]*self.errVel[1] + self.Kder[1]*derVel[1] + self.Kint[1]*self.errInt[1]
        des_a[2] = self.Kvel[2]*self.errVel[2] + self.Kder[2]*derVel[2] + self.Kint[2]*self.errInt[2]

        # des_a[0] = self.Kvel[0]*self.errVel[0] + self.Kpos[0]*errPos[0]
        # des_a[1] = self.Kvel[1]*self.errVel[1] + self.Kpos[1]*errPos[1]
        # des_a[2] = self.Kvel[2]*self.errVel[2] + self.Kpos[2]*errPos[2]
        # print(f'Pos offset: {errPos[2]}: Vel offset: {self.errVel[2]}')

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
        if self.landFlag:
            self.set_land()
        else:
            if self.odomFlag == True and self.lioFlag == True and self.controlFlag == True:
                self.mode()
                if(self.armed == 2 and self.offboard_mode == True):
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
                    thrust = np.maximum(-0.8, np.minimum(thrust, 0.8))
                    # print(yaw_ref, quat_des[3])

                    now = int(Clock().now().nanoseconds/1E3)

                    self.att_cmd.timestamp = now
                    self.att_cmd.q_d[0] = quat_des[3]
                    self.att_cmd.q_d[1] = quat_des[0]
                    self.att_cmd.q_d[2] = quat_des[1]
                    self.att_cmd.q_d[3] = quat_des[2]
                    
                    self.att_cmd.thrust_body[2] = thrust
                    print("thrust: {:.3f}".format(thrust))
                    # print(f"value: {des_a[0]:.3f}: {des_a[1]:.3f}:  {des_a[2]:.3f}")

                    self.publisher_attitude.publish(self.att_cmd)
                    

                elif self.arm_flag==False:
                    self.arm_uav()
                    # self.mode()
                    # print("Not armed yet")
                    pass

            else:
                print(f"Controller waiting: \nDLIO: {self.lioFlag}, \nFMUO: {self.odomFlag}, \nControl: {self.controlFlag}")
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
