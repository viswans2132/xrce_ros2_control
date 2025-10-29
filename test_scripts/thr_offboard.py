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
from geometry_msgs.msg import Twist

class OffboardControl(Node):

    def __init__(self):
        super().__init__('thrust_offboard_node')
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


        self.safetyRadius = 0.7 # meters
        self.vel_sp = np.array([0.0, 0.0, 0.0])

        self.des_vel = np.array([0.0, 0.0, 0.0])
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
        self.maxZVel = 0.5


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

        self.posSpSub = self.create_subscription(Twist, '/cmd_vel', self.sp_velocity_callback, laser_qos)
        self.publisher_attitude = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)

        print("Sleeping")
        time.sleep(2)
        print("Awake")
        self.mode()




 
    def vehicle_control_mode_callback(self, msg):
        # print(msg.flag_control_offboard_enabled)
        self.offboard_mode = msg.flag_control_offboard_enabled
        self.armed = msg.flag_armed
        # print(['offboard:',msg.flag_control_offboard_enabled])
 
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        #print("NAV_STATUS: ", msg.nav_state)
        #print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
        # self.armed = msg.arming_state
        # print(['armed:',msg.arming_state])


    def vehicle_position_callback(self, msg):
        self.cur_pose = np.array([msg.x,msg.y,msg.z])
        self.cur_vel = np.array([msg.vx,msg.vy,msg.vz])

    def vehicle_odometry_callback(self, msg):
        self.cur_ang = euler_from_quaternion([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])
        self.yaw = self.cur_ang[2]
        self.R = quaternion_matrix([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])[:-1, :-1]
        # print(self.R)
        # hello
        self.ang_vel = np.array([msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]])

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
        offboard_msg.position=False
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        offboard_msg.attitude=False
        offboard_msg.body_rate=False
        offboard_msg.thrust_and_torque=True
        # offboard_msg.actuator=False
        self.publisher_offboard_mode.publish(offboard_msg)


    def a_des(self):
        dt = Clock().now().nanoseconds/1E9 - self.pre_time
        self.pre_time = self.pre_time + dt
        if dt > 0.04:
            dt = 0.04


        self.time_from_start = Clock().now().nanoseconds/1E9 - self.offboard_time
        # self.pos_sp[0] = 2*np.cos(0.2*self.time_f rom_start)

        # curPos = self.vector2Arrays(self.cur_pose.pose.position)
        # desPos = self.vector2Arrays(self.sp.pose.position)
        # curVel = self.vector2Arrays(self.cur_vel.twist.linear)
        # curor = self.vector3Arrays(self.cur_pose.pose.orientation)

        errPos = self.cur_pose - self.pos_sp
        # print(np.linalg.norm(errPos))
        print("Error: {:.3f}, {:.3f}, {:.3f}".format(errPos[0], errPos[1], errPos[2]))
        errVel = self.cur_vel - self.vel_sp

        # errPos[1] = 0
        # errVel[1] = 0
        # errPos[0] = 0
        # errVel[0] = 0
        # self.errInt += errPos


        des_a = self.Kpos*errPos + self.Kvel*errVel + self.Kint*self.errInt
        # self.array2Vector3(errPos, self.data_out.position_error)
        # self.array2Vector3(errVel, self.data_out.velocity_error)
        # # self.array2Vector4(curor , self.data_out.curor)

        self.data_out.data = []
        self.data_out.data = [errPos[0], errPos[1], errPos[2], errVel[0], errVel[1], errVel[2], des_a[0], des_a[1], des_a[2]]

        if np.linalg.norm(des_a) > self.max_acc:
            des_a = (self.max_acc/np.linalg.norm(des_a))*des_a

        # print(des_a)

        return (des_a + self.gravity) 


    def acc2quat(self,des_a, des_yaw):
        xb_des = np.array([1, 0, 0.0])
        if np.linalg.norm(des_a) == 0.0:
            zb_des = np.array([0,0,1])
        else:    
            zb_des = -des_a / np.linalg.norm(des_a)
        yb_des = np.cross(zb_des, xb_des) / np.linalg.norm(np.cross(zb_des, xb_des))
        proj_xb_des = np.cross(yb_des, zb_des) / np.linalg.norm(np.cross(yb_des, zb_des))
       
        rotmat = np.transpose(np.array([proj_xb_des, yb_des, zb_des]))
        # print(np.linalg.det(rotmat))
        return rotmat



    def cmdloop_callback(self):
        self.mode()
        if(self.armed and self.offboard_mode):
            self.arm_flag = True
            self.cnt = self.cnt + 1

            des_a = self.a_des()

            # print(des_a)
            r_des = self.acc2quat(des_a, 0.0)
            # rot_44 = np.vstack((np.hstack((r_des,np.array([[0,0,0]]).T)), np.array([[0,0,0,1]])))
            e_R = 0.5*(r_des.T.dot(self.R) - self.R.T.dot(r_des))
            e_q = np.array([e_R[2,1], e_R[0,2], e_R[1,0]])

            # ang_sp = np.array([0.0, 0.0, 0.0])
            # ang_sp[0] = des_a[1]
            # ang_sp[1] = -des_a[0]
            # e_q = self.cur_ang - ang_sp
            e_qd = self.ang_vel
            # e_q[2] = e_q[2]
            # e_qd[2] = e_qd[2]


            des_t = self.Kq*e_q + self.Kqd*e_qd + np.cross(self.ang_vel, self.inertia*self.ang_vel)
            max_torq = np.array([0.1, 0.1, 0.02])
            des_t = np.maximum(-max_torq, np.minimum(max_torq, des_t))
            # des_t = self.Kq*e_q + self.Kqd*e_qd
            # des_t[0] = 0
            # des_t[2] = 0
            print("Thrust: {:.3f}, {:.3f}, {:.3f}".format(des_a[0], des_a[1], des_a[2]))
            print("Error Q: {:.3f}, {:.3f}, {:.3f}".format(e_q[0], e_q[1],e_q[2]))
            print("Torque: {:.3f}, {:.3f}, {:.3f}".format(des_t[0], des_t[1], des_t[2]))
            # hello

            # print(self.data_out.data.tolist())
            self.data_out.data =  self.data_out.data.tolist() + e_q.tolist() + e_qd.tolist() + des_t.tolist()
            # self.data_out.data =  + e_qd + des_t

            # des_a = des_a + self.gravity


            # quat_des = quaternion_from_matrix(rot_44)
           
            zb = r_des[:,2]
            # zb = np.array([0, 0, 1])
            thrust = self.norm_thrust_const * des_a.dot(zb) + 1
            # self.data_out.thrust = thrust
            thrust = np.maximum(-0.8, np.minimum(thrust, -0.1))
            # print(thrust)




            now = int(Clock().now().nanoseconds/1E3)

            self.thrust_cmd.timestamp_sample = now
            self.torque_cmd.timestamp_sample = now
            
            self.thrust_cmd.xyz[0] = 0.0
            self.thrust_cmd.xyz[1] = 0.0
            self.thrust_cmd.xyz[2] = thrust
            
            self.torque_cmd.xyz[0] = des_t[0]
            self.torque_cmd.xyz[1] = des_t[1]
            self.torque_cmd.xyz[2] = des_t[2]
            # self.torque_cmd.xyz[0] = 0.0
            # self.torque_cmd.xyz[1] = 0.0
            # self.torque_cmd.xyz[2] = 0.0
            
            self.publisher_thrust.publish(self.thrust_cmd)
            self.publisher_torque.publish(self.torque_cmd)
            self.publisher_data_out.publish(self.data_out)
            

        elif self.arm_flag==False:
            self.arm_uav()
            self.pos_sp = self.cur_pose + np.array([1.0, -1.0, -1.0])
            # print(self.pos_sp)
            # # self.mode()
            # print('Hello')













        

def main(args=None):
    rclpy.init(args=args)


    # print("Sleeping")
    # node = rclpy.create_node('offboard_node')
    # rate = node.create_rate(0.1)
    # rate.sleep()
    # print("Awake")

    offboard_control = OffboardControl()

    # while(offboard_control.armed!=2 or offboard_control.offboard_mode == False):         
    #     print(['Arming Status', offboard_control.armed])
    #     print(['Offboard Status', offboard_control.offboard_mode])
    #     offboard_control.arm_uav()
    #     time.sleep(0.1)
    print("True")

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
