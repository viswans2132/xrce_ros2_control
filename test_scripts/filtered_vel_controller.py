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
from sensor_msgs.msg import LaserScan, PointCloud2
import ros2_numpy
from tf_transformations import quaternion_matrix
from std_msgs.msg import Header

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

        self.des_pos = np.array([0.0, 0.0, -0.46])
        self.errInt = np.array([0.0, 0.0, 0.0])

        timer_period = 0.2  # 50 Hz
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.pos = np.array([0, 0, 0])
        self.cur_orien = np.zeros((4,1))
        self.dt = timer_period
        self.cnt = 0

        self.armed = False
        self.offboard_mode = False
        self.arm_flag = False
        self.odomFlag = False
        self.takeOffFlag = False
        self.consFlag = False

        self.A = np.array([])
        self.b = np.array([])

        # Velocity clipping
        self.maxXVel = 0.5
        self.maxYVel = 0.5
        self.maxZVel = 0.3

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
        # self.armed = msg.arming_state
        # print(['armed:', msg.arming_state])

    def vehicle_odometry_callback(self, msg):
        if self.odomFlag == False:
            # self.pos_sp[0] = msg.position[0]
            # self.pos_sp[1] = msg.position[1]
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

    def laserscan_callback(self, msg: LaserScan):
        # Step 1: Convert polar scan to (x,y) in LiDAR frame
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges)

        # Filter out invalid measurements
        mask = np.isfinite(ranges) & (ranges > 0.20) & (ranges < 2.75)
        ranges = ranges[mask]
        angles = angles[mask]

        # Cartesian (x, y, z=0) in LiDAR frame
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)

        points = np.vstack((x, y, z)).T   # Nx3 array

        # Step 2: Voxelization (same as before)
        voxel_size = 0.20
        discrete_coords = np.floor(points / voxel_size).astype(np.int32)
        _, unique_indices = np.unique(discrete_coords, axis=0, return_index=True)
        points = points[unique_indices]

        print(points.shape)

        # Step 3: Transform into drone body/world coordinates
        # Flip Y,Z like your original (depends on your convention)
        self.points_array = np.array([points[:,0], -points[:,1], -points[:,2]]).T
        # print(self.points_array)

        # Step 4: Flag and generate constraints
        if len(self.points_array) < 1:
            self.consFlag = False
        else:
            self.genConsMatrix()

        # # Step 5: Publish as PointCloud2
        points_xyzi = np.zeros((points.shape[0], 4), dtype=np.float32)
        points_xyzi[:, :3] = points
        points_xyzi[:, 3] = 1.0   # or any fake intensity
        structured_array = np.zeros(points.shape[0], dtype=[('x', np.float32),('y', np.float32),('z', np.float32),('intensity', np.float32)])
        structured_array['x'] = points_xyzi[:, 0]
        structured_array['y'] = points_xyzi[:, 1]
        structured_array['z'] = points_xyzi[:, 2]
        structured_array['intensity'] = points_xyzi[:, 3]

        points = np.array(points, dtype=np.float32)
        assert points.shape[1] == 3

        pcl = ros2_numpy.msgify(PointCloud2, structured_array)


        # pcl = ros2_numpy.msgify(PointCloud2, {'xyz': points.astype(np.float32)})
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()  # current ROS 2 timestamp
        header.frame_id = 'laser'                # must exist in TF tree
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
            self.b = -1.5 * (np.sum(points_xy**2, axis=1) - self.safetyRadius**2)

            if self.takeOffFlag:
                self.consFlag = True


    def safety_filter(self, desVel):
        print("Safety ON")
        P = np.eye(2)
        u = cp.Variable(2)
        print(f"{desVel[0]:.2f}")

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

            print(f"{val[0]:.2f}")

            return val
        
        else:
            return np.zeros(desVel.shape)





    def cmdloop_callback(self):
        if self.odomFlag:
            self.mode()
            # print([self.armed ,self.offboard_mode])

            if (self.armed and self.offboard_mode):
                self.arm_flag = True

                if not self.takeOffFlag:
                    v_cmd = np.array([0.0, 0.0, -5.0])

                else:
                    # print("Publishing")

                    error = self.des_pos - self.pos
                    # self.errInt = self.errInt + error*self.dt

                    vel_cmd = 0.5*error

                    if self.consFlag:
                        vel_cmd[:2] = self.safety_filter(vel_cmd[:2])

                    v_min = np.array([-0.2, -0.2, -1.0])
                    v_max = np.array([0.2, 0.2, 0.2])
                    v_cmd = np.maximum(v_min, np.minimum(v_max, vel_cmd))
                    # print(f"Error: {error[0]:.2f}, {error[1]:.2f}, {error[2]:.2f}")
                    # print(f"Position: {self.pos[0]:.2f}, {self.pos[1]:.2f}, {self.pos[2]:.2f}")
                    # print(f"Command: {v_cmd[0]:.2f}, {v_cmd[1]:.2f}, {v_cmd[2]:.2f}")


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
