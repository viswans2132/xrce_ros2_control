#!/usr/bin/env python3
import rclpy
import numpy as np
import cvxpy as cp
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
from px4_msgs.msg import VehicleAttitudeSetpoint
from sensor_msgs.msg import LaserScan, PointCloud2
import ros2_numpy
from tf_transformations import quaternion_matrix, euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

class OffboardControl(Node):

    def __init__(self):
        super().__init__('velocity_offboard_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        laser_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )


        self.safetyRadius = 1.0 # meters
        self.pos_sp = np.array([0.0, 0.0])

        self.des_pos = np.array([0.0, 0.0, -1.46])
        self.errInt = np.array([0.0, 0.0, 0.0])

        self.lidar_rel_pos = np.array([0.12, 0.0, 0.26])

        timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
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
        self.relayFlag = True
        self.lioFlag = True
        self.controlFlag = True
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

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile)

        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)

        self.mode_sub = self.create_subscription(
            VehicleControlMode,
            '/fmu/out/vehicle_control_mode',
            self.vehicle_control_mode_callback,
            qos_profile)

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laserscan_callback,
            laser_qos)

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
        self.publisher_points = self.create_publisher(PointCloud2, '/reduced_points', laser_qos)

        self.posSpSub = self.create_subscription(PoseStamped, '/ref', self.sp_position_callback, laser_qos)
        self.publisher_attitude = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)

        print("Sleeping")
        time.sleep(2)
        print("Awake")
        self.mode()


    def sp_position_callback(self, msg):
        self.pos_sp[0] = msg.pose.position.x
        self.pos_sp[1] = -msg.pose.position.y
        # self.pos_sp[2] = -msg.pose.position.z
        # quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        # self.yaw_sp = -euler_from_quaternion(quat)[2]
        print("New setpoint received")
        print(self.pos_sp)

    def vehicle_control_mode_callback(self, msg):
        self.offboard_mode = msg.flag_control_offboard_enabled
        self.armed = msg.flag_armed
        # print(['offboard:', msg.flag_control_offboard_enabled])

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state

    def vehicle_odometry_callback(self, msg):
        if self.odomFlag == False:
            # self.pos_sp[0] = msg.position[0]
            # self.pos_sp[1] = msg.position[1]
            self.odomFlag = True
        self.cur_pos = np.array([msg.position[0],msg.position[1],msg.position[2]])
        self.cur_vel = np.array([msg.velocity[0],msg.velocity[1],msg.velocity[2]])
        self.cur_orien = np.array([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])
        self.yaw = euler_from_quaternion([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])[2]
        if self.cur_pos[2] < -0.45:
            if not self.takeOffFlag:
                print("Takeoff detected")                
            self.takeOffFlag = True
            self.des_pos[0] = self.pos_sp[0]
            self.des_pos[1] = self.pos_sp[1]

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
        voxel_size = 0.50
        discrete_coords = np.floor(points / voxel_size).astype(np.int32)
        _, unique_indices = np.unique(discrete_coords, axis=0, return_index=True)
        points = points[unique_indices] 
        self.points_array = np.array([points[:,0], -points[:,1], -points[:,2]]).T



        trans_points = points  + self.lidar_rel_pos
        rot = quaternion_matrix(self.cur_orien)[:-1, :-1]

        rot_points = trans_points@rot + np.array([self.cur_pos[0], -self.cur_pos[1], -self.cur_pos[2]])

        # print(rot_points.shape)

        # Step 3: Transform into drone body/world coordinates
        # Flip Y,Z like your original (depends on your convention)


        # print(self.points_array)

        # Step 4: Flag and generate constraints
        if len(self.points_array) < 1:
            self.consFlag = False
        else:
            self.genConsMatrix()

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

        # Rotate points
        rotated_points = (R @ self.points_array.T).T
        # Keep only x, y for constraints
        points_xy = rotated_points[:, :2]
        # print(self.cur_orien)
    
        # print(R)

        # print(points_xy)


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

        if len(self.A) == len(self.b):
            constraints = [self.A@u >= self.b]
            prob = cp.Problem(cp.Minimize(cp.quad_form(u-desVel, P)), constraints)

            try:
                result = prob.solve()
                val = u.value
                # print(f"{val[0]:.2f}")

                try:
                    val = np.maximum(-np.array([self.maxXVel, self.maxYVel]), np.minimum(np.array([self.maxXVel, self.maxYVel]), val))
                except TypeError:
                    print("Safety filter returned None")
                    val = np.array([0.0, 0.0])

            except cp.error.SolverError:
                print("Safety filter: solver error")
                val =  np.array([0.0, 0.0])


            return val
        
        else:
            return np.zeros(desVel.shape)



    def a_des(self):
        R = np.array([[np.cos(self.yaw), np.sin(self.yaw), 0], [-np.sin(self.yaw), np.cos(self.yaw), 0], [0, 0, 1]])
        cur_vel_W = R.T.dot(self.cur_vel)

        errPos = self.cur_pos - self.des_pos

        if np.linalg.norm(errPos[2]) < 0.05:
            self.trajFlag = True

        if self.trajFlag == True:
            self.time_from_start = Clock().now().nanoseconds/1E9 - self.offboard_time
            # self.pos_sp[0] = 0.5*np.cos(0.4*self.time_from_start)
            # self.pos_sp[1] = 0.5*np.sin(0.4*self.time_from_start)


        desVel = self.Kpos * errPos

        if self.consFlag:
            desVel[:2] = self.safety_filter(desVel[:2])
        
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
            if self.odomFlag and self.lioFlag and self.controlFlag:
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
                print("Odometry not received")


def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
