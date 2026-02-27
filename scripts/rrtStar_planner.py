#!/usr/bin/env python3
"""
Single-file ROS 2 (rclpy) node:

- Informed RRT* planner in 3D (x,y,z) with vertical cylindrical obstacles loaded from YAML.
- Subscribes:
    * /odom            (nav_msgs/Odometry)        -> start position
    * /goal_pose       (geometry_msgs/PoseStamped)-> new goal triggers ONE planning run
- Publishes:
    * /planned_path    (nav_msgs/Path)            -> full planned path with yaw "facing forward"
    * /ref_pose        (geometry_msgs/PoseStamped)-> continuously at fixed rate, progressing along path
        based on how far the UAV has moved (ds from odom), with a lookahead point

No replanning timer. It replans ONLY when a new goal PoseStamped arrives.

Run (example):
  python3 informed_rrt_planner.py --ros-args \
    -p obstacles_yaml:="$(pwd)/obstacles.yaml" \
    -p bounds_min:="[-20.0, -20.0, 0.0]" \
    -p bounds_max:="[20.0, 20.0, 6.0]" \
    -p world_frame:=map

Publish a goal:
  ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{
    header: {frame_id: map},
    pose: {position: {x: 8.0, y: 2.0, z: 2.5}, orientation: {w: 1.0}}
  }"
"""

import math
import time
import yaml
import random
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry


# ============================================================
# Data
# ============================================================

@dataclass
class CylinderObs:
    x: float
    y: float
    r: float
    z_min: float
    z_max: float


@dataclass
class RRTNode:
    p: np.ndarray
    parent: int
    cost: float


def _dist(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a - b))


# ============================================================
# Informed RRT*
# ============================================================

class InformedRRTStar:
    """
    Informed RRT* in 3D position space with vertical cylinder obstacles.

    Notes:
    - Straight-line edges in R^3 (no dynamics).
    - Cylinders block only within [z_min, z_max].
    - Intended for generating waypoint paths, tracked by your controller (MPC/CBF/etc.).
    """

    def __init__(
        self,
        bounds_min: np.ndarray,
        bounds_max: np.ndarray,
        obstacles: List[CylinderObs],
        inflate: float = 0.0,
        step_size: float = 0.6,
        neighbor_radius: float = 1.8,
        max_iters: int = 1000,
        goal_sample_rate: float = 0.12,
        goal_tolerance: float = 0.7,
        segment_check_res: float = 0.15,
        rng_seed: Optional[int] = 7,
    ):
        self.bmin = np.array(bounds_min, dtype=float)
        self.bmax = np.array(bounds_max, dtype=float)
        self.obstacles = obstacles
        self.inflate = float(inflate)

        self.step_size = float(step_size)
        self.neighbor_radius = float(neighbor_radius)
        self.max_iters = int(max_iters)
        self.goal_sample_rate = float(goal_sample_rate)
        self.goal_tolerance = float(goal_tolerance)
        self.segment_check_res = float(segment_check_res)

        self.rng = random.Random(rng_seed)

        self.nodes: List[RRTNode] = []
        self.best_goal_idx: Optional[int] = None
        self.best_cost: float = float("inf")

        # Informed sampling state
        self.c_min = 0.0
        self.x_center = None
        self.C = None  # basis
        self.a1 = None

    def _in_bounds(self, p: np.ndarray) -> bool:
        return bool(np.all(p >= self.bmin) and np.all(p <= self.bmax))

    def _point_in_collision(self, p: np.ndarray) -> bool:
        x, y, z = float(p[0]), float(p[1]), float(p[2])
        for o in self.obstacles:
            if z < o.z_min or z > o.z_max:
                continue
            dx = x - o.x
            dy = y - o.y
            rr = o.r + self.inflate
            if dx * dx + dy * dy <= rr * rr:
                return True
        return False

    def _segment_in_collision(self, a: np.ndarray, b: np.ndarray) -> bool:
        L = _dist(a, b)
        if L < 1e-9:
            return self._point_in_collision(a)
        n = max(2, int(math.ceil(L / self.segment_check_res)) + 1)
        for i in range(n):
            t = i / (n - 1)
            p = a * (1.0 - t) + b * t
            if self._point_in_collision(p):
                return True
        return False

    def _sample_uniform(self) -> np.ndarray:
        return np.array([
            self.rng.uniform(self.bmin[0], self.bmax[0]),
            self.rng.uniform(self.bmin[1], self.bmax[1]),
            self.rng.uniform(self.bmin[2], self.bmax[2]),
        ], dtype=float)

    def _setup_informed(self, start: np.ndarray, goal: np.ndarray):
        self.c_min = _dist(start, goal)
        self.x_center = 0.5 * (start + goal)

        a1 = (goal - start) / max(self.c_min, 1e-9)
        self.a1 = a1

        # Build orthonormal basis C with first column = a1
        v2 = np.array([1.0, 0.0, 0.0], dtype=float)
        if abs(np.dot(v2, a1)) > 0.9:
            v2 = np.array([0.0, 1.0, 0.0], dtype=float)
        b2 = v2 - np.dot(v2, a1) * a1
        b2 /= max(np.linalg.norm(b2), 1e-9)
        b3 = np.cross(a1, b2)
        b3 /= max(np.linalg.norm(b3), 1e-9)
        self.C = np.column_stack((a1, b2, b3))

    def _sample_informed(self) -> np.ndarray:
        if not math.isfinite(self.best_cost):
            return self._sample_uniform()

        c_best = self.best_cost
        c_min = max(self.c_min, 1e-9)

        r1 = c_best / 2.0
        r2 = math.sqrt(max(c_best * c_best - c_min * c_min, 0.0)) / 2.0
        r3 = r2

        # sample unit ball
        while True:
            x = np.array([self.rng.uniform(-1, 1),
                          self.rng.uniform(-1, 1),
                          self.rng.uniform(-1, 1)], dtype=float)
            if float(np.dot(x, x)) <= 1.0:
                break

        L = np.diag([r1, r2, r3])
        p = self.C @ (L @ x) + self.x_center
        return p

    def _nearest(self, p: np.ndarray) -> int:
        d2 = [float(np.dot(n.p - p, n.p - p)) for n in self.nodes]
        return int(np.argmin(d2))

    def _near_indices(self, p: np.ndarray) -> List[int]:
        r2 = self.neighbor_radius * self.neighbor_radius
        out = []
        for i, n in enumerate(self.nodes):
            if float(np.dot(n.p - p, n.p - p)) <= r2:
                out.append(i)
        return out

    def _steer(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        v = b - a
        L = float(np.linalg.norm(v))
        if L < 1e-9:
            return a.copy()
        step = min(self.step_size, L)
        return a + (v / L) * step

    def plan(self, start: np.ndarray, goal: np.ndarray) -> Optional[List[np.ndarray]]:
        start = np.array(start, dtype=float).copy()
        goal = np.array(goal, dtype=float).copy()

        if not self._in_bounds(start) or not self._in_bounds(goal):
            return None
        if self._point_in_collision(start) or self._point_in_collision(goal):
            return None

        self.nodes = [RRTNode(p=start, parent=-1, cost=0.0)]
        self.best_goal_idx = None
        self.best_cost = float("inf")
        self._setup_informed(start, goal)

        for _ in range(self.max_iters):
            # goal bias
            if self.rng.random() < self.goal_sample_rate:
                x_rand = goal
            else:
                x_rand = self._sample_informed()

            if not self._in_bounds(x_rand) or self._point_in_collision(x_rand):
                continue

            i_near = self._nearest(x_rand)
            x_new = self._steer(self.nodes[i_near].p, x_rand)

            if not self._in_bounds(x_new) or self._point_in_collision(x_new):
                continue
            if self._segment_in_collision(self.nodes[i_near].p, x_new):
                continue

            near = self._near_indices(x_new)

            # choose best parent
            best_parent = i_near
            best_cost = self.nodes[i_near].cost + _dist(self.nodes[i_near].p, x_new)

            for j in near:
                cand = self.nodes[j].cost + _dist(self.nodes[j].p, x_new)
                if cand < best_cost and not self._segment_in_collision(self.nodes[j].p, x_new):
                    best_cost = cand
                    best_parent = j

            new_idx = len(self.nodes)
            self.nodes.append(RRTNode(p=x_new, parent=best_parent, cost=best_cost))

            # rewire
            for j in near:
                cand = self.nodes[new_idx].cost + _dist(self.nodes[new_idx].p, self.nodes[j].p)
                if cand < self.nodes[j].cost and not self._segment_in_collision(self.nodes[new_idx].p, self.nodes[j].p):
                    self.nodes[j].parent = new_idx
                    self.nodes[j].cost = cand

            # connect to goal
            if _dist(x_new, goal) <= self.goal_tolerance and not self._segment_in_collision(x_new, goal):
                g_cost = self.nodes[new_idx].cost + _dist(x_new, goal)
                if g_cost < self.best_cost:
                    g_idx = len(self.nodes)
                    self.nodes.append(RRTNode(p=goal.copy(), parent=new_idx, cost=g_cost))
                    self.best_goal_idx = g_idx
                    self.best_cost = g_cost

        if self.best_goal_idx is None:
            return None

        # backtrack
        path = []
        idx = self.best_goal_idx
        while idx != -1:
            path.append(self.nodes[idx].p.copy())
            idx = self.nodes[idx].parent
        path.reverse()

        # shortcut smoothing
        path = self._shortcut_smooth(path, iters=120)
        return path

    def _shortcut_smooth(self, path: List[np.ndarray], iters: int = 100) -> List[np.ndarray]:
        if len(path) < 3:
            return path
        out = path[:]
        for _ in range(iters):
            if len(out) < 3:
                break
            i = self.rng.randint(0, len(out) - 3)
            j = self.rng.randint(i + 2, len(out) - 1)
            if not self._segment_in_collision(out[i], out[j]):
                out = out[: i + 1] + out[j:]
        return out


# ============================================================
# Node: goal-triggered planning + ref_pose tracking
# ============================================================

class InformedRRTPlannerNode(Node):
    def __init__(self):
        super().__init__("informed_rrt_planner")

        # --- Minimal parameters (only the essentials) ---
        self.declare_parameter("obstacles_yaml", "")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("ref_topic", "/ref_pose")

        self.declare_parameter("bounds_min", [-10.0, -10.0, 0.0])
        self.declare_parameter("bounds_max", [10.0, 10.0, 5.0])

        # One safety margin (optional). If YAML has inflate_radius, we take max(yaml, param).
        self.declare_parameter("inflate_radius", 0.5)

        # --- Hardcoded tracking settings (no params) ---
        self._ref_dt = 0.05          # 20 Hz ref output
        self._lookahead = 0.8        # meters ahead along the path
        self._goal_change_eps = 1e-3 # ignore negligible goal updates

        # --- State ---
        self._have_odom = False
        self._p_world = np.zeros(3, dtype=float)
        self._yaw = 0.0
        self._yaw_sp = 0.0

        self._goal_world = np.zeros(3, dtype=float)
        self._last_goal_world: Optional[np.ndarray] = None

        # Stored path polyline and arc-length
        self._path_pts: Optional[np.ndarray] = None   # (N,3)
        self._path_s: Optional[np.ndarray] = None     # (N,)
        self._path_total: float = 0.0
        self._s_prog: float = 0.0
        self._prev_odom_p: Optional[np.ndarray] = None

        self._wp_idx = 0
        self._wp_reach_thresh = 0.10  # meters

        # Obstacles + planner
        obstacles, yaml_inflate = self._load_obstacles_yaml(self.get_parameter("obstacles_yaml").value)
        inflate = float(self.get_parameter("inflate_radius").value)
        if yaml_inflate is not None:
            inflate = max(inflate, float(yaml_inflate))

        bmin = np.array(self.get_parameter("bounds_min").value, dtype=float)
        bmax = np.array(self.get_parameter("bounds_max").value, dtype=float)

        self.planner = InformedRRTStar(
            bounds_min=bmin,
            bounds_max=bmax,
            obstacles=obstacles,
            inflate=inflate,
            # planner “good defaults” (hardcoded to avoid parameter sprawl)
            step_size=0.6,
            neighbor_radius=1.8,
            max_iters=1000,
            goal_sample_rate=0.12,
            goal_tolerance=0.7,
            segment_check_res=0.15,
            rng_seed=7,
        )

        # QoS
        qos_odom = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_goal = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Subs
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self._odom_cb, qos_odom)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self._veh_odom_cb, qos_odom)
        self.create_subscription(PoseStamped, self.get_parameter("goal_topic").value, self._goal_cb, qos_goal)

        # Pubs
        self.path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, 10)
        self.ref_pub = self.create_publisher(PoseStamped, self.get_parameter("ref_topic").value, 10)

        # Timer: publish ref_pose continuously (path must exist)
        self.create_timer(self._ref_dt, self._ref_timer_cb)

        self.get_logger().info(
            f"Loaded {len(obstacles)} obstacles. Planning bounds min={bmin.tolist()} max={bmax.tolist()} inflate={inflate:.3f}."
        )
        self.get_logger().info("Will plan ONLY when a new /goal_pose arrives, and will publish /ref_pose continuously once a path exists.")

    # -------------------------
    # YAML loading
    # -------------------------

    def _load_obstacles_yaml(self, yaml_path: str) -> Tuple[List[CylinderObs], Optional[float]]:
        if not yaml_path:
            self.get_logger().warn("No obstacles_yaml provided; planning with no obstacles.")
            return [], None

        try:
            with open(yaml_path, "r") as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f"Failed to read YAML '{yaml_path}': {e}")
            return [], None

        obs_list: List[CylinderObs] = []
        for o in (data.get("obstacles", []) or []):
            if o.get("type", "cylinder") != "cylinder":
                continue
            try:
                obs_list.append(CylinderObs(
                    x=float(o["x"]),
                    y=float(o["y"]),
                    r=float(o["radius"]),
                    z_min=float(o.get("z_min", -1e9)),
                    z_max=float(o.get("z_max", 1e9)),
                ))
            except Exception as e:
                self.get_logger().warn(f"Skipping malformed obstacle {o}: {e}")

        inflate = data.get("inflate_radius", None)
        return obs_list, inflate

    # -------------------------
    # Callbacks
    # -------------------------

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._p_world[:] = [p.x, p.y, p.z]
        q = [msg.q[0], msg.q[1], -msg.q[2], msg.q[3]]
        siny_cosp = 2.0 * (q[3] * q[2] + q[0] * q[1])
        cosy_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
        self._yaw = np.arctan2(siny_cosp, cosy_cosp)
        self._have_odom = True

    def _veh_odom_cb(self, msg: VehicleOdometry):
        p = msg.position
        self._p_world[:] = [p[1], p[0], -p[2]]
        q = [msg.q[1], msg.q[2], msg.q[3], msg.q[0]]
        siny_cosp = 2.0 * (q[3] * q[2] + q[0] * q[1])
        cosy_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
        self._yaw = np.arctan2(siny_cosp, cosy_cosp)
        self._have_odom = True

    def _goal_cb(self, msg: PoseStamped):
        g = msg.pose.position
        goal = np.array([g.x, g.y, g.z], dtype=float)

        if self._last_goal_world is not None:
            if float(np.linalg.norm(goal - self._last_goal_world)) < self._goal_change_eps:
                return

        self._wp_idx = 0

        self._goal_world[:] = goal
        self._last_goal_world = goal.copy()

        if not self._have_odom:
            self.get_logger().warn("Received goal but no /odom yet; waiting for odometry.")
            return

        self._plan_and_publish()

    # -------------------------
    # Planning + path publication
    # -------------------------

    def _plan_and_publish(self):
        start = self._p_world.copy()
        goal = self._goal_world.copy()

        # Optionally clamp start to bounds (prevents instant failure if odom z slightly below z_min)
        start = np.maximum(start, self.planner.bmin)
        start = np.minimum(start, self.planner.bmax)

        # Early debug messages for instant failures
        if not self.planner._in_bounds(start):
            self.get_logger().warn(f"Start out of bounds: {start.tolist()}")
            return
        if not self.planner._in_bounds(goal):
            self.get_logger().warn(f"Goal out of bounds: {goal.tolist()}")
            return
        if self.planner._point_in_collision(start):
            self.get_logger().warn(f"Start in collision: {start.tolist()} inflate={self.planner.inflate}")
            return
        if self.planner._point_in_collision(goal):
            self.get_logger().warn(f"Goal in collision: {goal.tolist()} inflate={self.planner.inflate}")
            return

        t0 = time.time()
        path_pts = self.planner.plan(start, goal)
        dt_ms = (time.time() - t0) * 1000.0

        if path_pts is None or len(path_pts) < 2:
            self.get_logger().warn(f"Planning failed (took {dt_ms:.1f} ms). start={start.tolist()} goal={goal.tolist()}")
            return

        # Store path polyline + arc-length for ref_pose tracking
        self._store_path_for_tracking(path_pts)

        # Build and publish Path with forward-facing yaw
        msg = Path()
        msg.header.frame_id = self.get_parameter("world_frame").value
        msg.header.stamp = self.get_clock().now().to_msg()

        yaws = self._compute_forward_yaws(path_pts, smooth_window=7)

        for k, p in enumerate(path_pts):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(p[0])
            ps.pose.position.y = float(p[1])
            ps.pose.position.z = float(p[2])

            qx, qy, qz, qw = self._yaw_to_quat(float(yaws[k]))
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw

            msg.poses.append(ps)

        self.path_pub.publish(msg)
        self.get_logger().info(
            f"Planned path: {len(path_pts)} waypoints, cost≈{self.planner.best_cost:.2f}, time={dt_ms:.1f} ms"
        )

    # -------------------------
    # Ref pose publisher (timer)
    # -------------------------
    def _ref_timer_cb(self):
        # Need odom + a stored path
        if not self._have_odom:
            return
        if self._path_pts is None or self._path_pts.shape[0] < 1:
            return

        p_now = self._p_world.copy()

        # Clamp index to valid range
        n = int(self._path_pts.shape[0])
        if self._wp_idx >= n:
            self._wp_idx = n - 1

        # Current target waypoint
        p_tgt = self._path_pts[self._wp_idx]
        # print(float(np.linalg.norm(p_now - p_tgt)))

        # If we're close enough, advance to next waypoint (but don't run past the goal)
        if float(np.linalg.norm(p_now - p_tgt)) <= self._wp_reach_thresh:
            if self._wp_idx < n - 1:
                self._wp_idx += 1
                p_tgt = self._path_pts[self._wp_idx]

        err = p_tgt - p_now
        dist = np.linalg.norm(err)
        if dist > 0.3:
            p_tgt = p_now + (0.3 * err / dist)


        if dist > 0.1:
            d = err
            dx, dy = float(d[0]), float(d[1])
            yaw = math.atan2(dy, dx) if (abs(dx) + abs(dy)) > 1e-9 else 0.0

            errYaw = yaw - self._yaw

            if np.abs(errYaw) > np.pi and np.abs(self._yaw) < 3.1:
                errYaw = np.sign(errYaw)*(np.abs(errYaw) - 2*np.pi)

            self._yaw_sp = self._yaw + np.clip(errYaw, -0.3, 0.3)


        qx, qy, qz, qw = self._yaw_to_quat(self._yaw_sp)

        # Publish ref pose
        msg = PoseStamped()
        msg.header.frame_id = self.get_parameter("world_frame").value
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = float(p_tgt[0])
        msg.pose.position.y = float(p_tgt[1])
        msg.pose.position.z = float(p_tgt[2])

        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.ref_pub.publish(msg)
    # -------------------------
    # Path tracking utilities
    # -------------------------

    def _store_path_for_tracking(self, path_pts: List[np.ndarray]):
        P = np.asarray(path_pts, dtype=float)
        ds = np.linalg.norm(P[1:] - P[:-1], axis=1)
        s = np.concatenate(([0.0], np.cumsum(ds)))

        self._path_pts = P
        self._path_s = s
        self._path_total = float(s[-1])

        self._s_prog = 0.0
        self._prev_odom_p = self._p_world.copy()

    def _sample_path_at_s(self, s_query: float) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if self._path_pts is None or self._path_s is None or self._path_pts.shape[0] < 2:
            return None, None

        s_query = float(np.clip(s_query, 0.0, self._path_total))

        k = int(np.searchsorted(self._path_s, s_query, side="right") - 1)
        k = max(0, min(k, self._path_pts.shape[0] - 2))

        s0 = float(self._path_s[k])
        s1 = float(self._path_s[k + 1])
        p0 = self._path_pts[k]
        p1 = self._path_pts[k + 1]

        seg = p1 - p0
        seg_len = float(np.linalg.norm(seg))
        if seg_len < 1e-9 or abs(s1 - s0) < 1e-9:
            return p0.copy(), np.array([1.0, 0.0, 0.0], dtype=float)

        a = (s_query - s0) / (s1 - s0)
        p_ref = p0 * (1.0 - a) + p1 * a
        t_hat = seg / seg_len
        return p_ref, t_hat

    # -------------------------
    # Yaw utilities for Path
    # -------------------------

    def _yaw_to_quat(self, yaw: float) -> Tuple[float, float, float, float]:
        half = 0.5 * float(yaw)
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def _compute_forward_yaws(self, path_pts: List[np.ndarray], smooth_window: int = 7) -> np.ndarray:
        n = len(path_pts)
        if n == 0:
            return np.array([], dtype=float)
        if n == 1:
            return np.array([0.0], dtype=float)

        yaws = np.zeros(n, dtype=float)
        for k in range(n - 1):
            d = path_pts[k + 1] - path_pts[k]
            dx, dy = float(d[0]), float(d[1])
            if abs(dx) < 1e-9 and abs(dy) < 1e-9:
                yaws[k] = yaws[k - 1] if k > 0 else 0.0
            else:
                yaws[k] = math.atan2(dy, dx)
        yaws[-1] = yaws[-2]

        # unwrap
        yaws = np.unwrap(yaws)

        # moving average smoothing
        w = int(max(1, smooth_window))
        if w > 1:
            if w % 2 == 0:
                w += 1
            pad = w // 2
            ypad = np.pad(yaws, (pad, pad), mode="edge")
            kernel = np.ones(w, dtype=float) / float(w)
            yaws = np.convolve(ypad, kernel, mode="valid")

        # wrap to [-pi, pi]
        yaws = (yaws + math.pi) % (2.0 * math.pi) - math.pi
        return yaws


# ============================================================
# Main
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = InformedRRTPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()