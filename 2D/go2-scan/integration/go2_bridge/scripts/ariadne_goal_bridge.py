#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ARiADNE 航点 → SCAN-Planner 参考路径桥（A2 架构唯一自研胶水）。

官方契约：rl_planner 输出 /way_point(PointStamped, ≤2.5Hz)，由下游 waypoint
follower 导航。本机用 SCAN-Planner navi_mode=3 替代 follower：
每条 /initial_path(nav_msgs/Path) 触发一次全局参考重建，走完一段进 WAIT_TARGET。
由此推出四条接口纪律（源码依据见 scan_replan_fsm.cpp pathCallback）：
  1. Path 只需两点 [当前位置, 航点]——SCAN 对稀疏路径自动插中间点
  2. poses[0] 必须是轨迹起点（SCAN 把首点当 trajectory start）
  3. z 携带地面高度；二维模式由SCAN在接收目标时采用当前机体Z，
     高程模式使用目标格地面高度加机体离地高度
  4. 必须去重：way_point 以固定频率重复发布同一目标，
     不去重会反复触发 SCAN 整体重规划
frame_id 用 world（SCAN 全局系；数值与 map 恒等，由 map→world 静态 TF 桥保证）。
"""

import math
import time
import copy
import json
from collections import deque
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Bool, Float64, String


class WaypointBridge(object):
    def __init__(self):
        self.repub_dist = float(rospy.get_param('~repub_dist', 0.5))
        self.target_min_clearance = float(rospy.get_param('~target_min_clearance', 0.30))
        self.target_min_robot_distance = float(rospy.get_param(
            '~target_min_robot_distance', 0.0))
        self.target_search_radius = float(rospy.get_param('~target_search_radius', 5.0))
        self.use_elevation = bool(rospy.get_param('~use_elevation', False))
        self.elevation_cloud_topic = rospy.get_param(
            '~elevation_cloud_topic', '/local_elevation_cloud')
        self.elevation_max_distance = float(rospy.get_param(
            '~elevation_max_distance', 0.20))
        self.body_pose_topic = rospy.get_param('~body_pose_topic', '/LIO/odom_vehicle')
        if self.target_min_clearance < 0.0:
            raise ValueError('target_min_clearance must be non-negative')
        if self.target_min_robot_distance < 0.0:
            raise ValueError('target_min_robot_distance must be non-negative')
        if self.target_search_radius < self.target_min_clearance:
            raise ValueError('target_search_radius must be >= target_min_clearance')
        self.robot_xy = None
        self.elevation_tree = None
        self.elevation_points = None
        self.elevation_np = None
        self.elevation_reader = None
        self.elevation_tree_type = None
        # Path.z carries the floor reference. SCAN samples current body Z for
        # each goal in 2-D mode and uses path height only in elevation mode.
        self.floor_z_ref = float(rospy.get_param('~floor_z_ref', 0.0))
        self.last_sent = None
        self.projected_map = None
        self.paused = False
        # latch 不开：latch 会让迟连接的订阅者收到旧 Path 触发意外重规划
        self.path_pub = rospy.Publisher('/initial_path', Path, queue_size=1)
        self.valid_pub = rospy.Publisher('/ariadne/bridge/target_valid', Bool,
                                         queue_size=1, latch=True)
        self.diag_pub = rospy.Publisher('/ariadne/bridge/diagnostic', String, queue_size=10)
        rospy.Subscriber(self.body_pose_topic, Odometry, self.odom_cb, queue_size=1)
        if self.use_elevation:
            import numpy as np
            from scipy.spatial import cKDTree
            from sensor_msgs import point_cloud2
            from sensor_msgs.msg import PointCloud2
            self.elevation_np = np
            self.elevation_reader = point_cloud2
            self.elevation_tree_type = cKDTree
            self.elevation_points = np.empty((0, 3), dtype=np.float64)
            rospy.Subscriber(self.elevation_cloud_topic, PointCloud2,
                             self.elevation_cb, queue_size=1)
        rospy.Subscriber('/floor_context/z_ref', Float64, self.floor_z_cb, queue_size=1)
        rospy.Subscriber('/way_point', PointStamped, self.waypoint_cb, queue_size=1)
        rospy.Subscriber('/projected_map', OccupancyGrid, self.map_cb, queue_size=1)
        rospy.Subscriber('/ariadne/lifecycle/pause', Bool, self.pause_cb, queue_size=1)
        rospy.Subscriber('/ariadne/lifecycle/reset_for_floor', Bool, self.reset_cb, queue_size=1)
        self.valid_pub.publish(Bool(False))
        rospy.loginfo('[ariadne_goal_bridge] floor_z_ref=%.3f elevation=%s repub=%.2fm '
                      'target_2d_clearance>%.2fm target_distance>%.2fm search=%.2fm',
                      self.floor_z_ref, self.use_elevation,
                      self.repub_dist,
                      self.target_min_clearance, self.target_min_robot_distance,
                      self.target_search_radius)

    def publish_diag(self, event, **fields):
        fields['event'] = event
        self.diag_pub.publish(String(json.dumps(fields, ensure_ascii=False, sort_keys=True)))

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.robot_xy = (p.x, p.y)

    def elevation_cb(self, msg):
        points = self.elevation_np.asarray(list(self.elevation_reader.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)),
            dtype=self.elevation_np.float64)
        if points.size == 0:
            self.elevation_points = self.elevation_np.empty(
                (0, 3), dtype=self.elevation_np.float64)
            self.elevation_tree = None
            return
        self.elevation_points = points.reshape((-1, 3))
        self.elevation_tree = self.elevation_tree_type(self.elevation_points[:, :2])

    def ground_z_at(self, x, y):
        if not self.use_elevation:
            return self.floor_z_ref
        if self.elevation_tree is None:
            return None
        distance, index = self.elevation_tree.query((x, y), k=1)
        if distance > self.elevation_max_distance:
            return None
        z = float(self.elevation_points[index, 2])
        return z if math.isfinite(z) else None

    def floor_z_cb(self, msg):
        self.floor_z_ref = msg.data

    def map_cb(self, msg):
        self.projected_map = msg

    @staticmethod
    def map_cell(msg, x, y):
        res = msg.info.resolution
        ix = int(math.floor((x - msg.info.origin.position.x) / res))
        iy = int(math.floor((y - msg.info.origin.position.y) / res))
        if ix < 0 or iy < 0 or ix >= msg.info.width or iy >= msg.info.height:
            return None
        return ix, iy

    @staticmethod
    def cell_center(msg, ix, iy):
        res = msg.info.resolution
        return (msg.info.origin.position.x + (ix + 0.5) * res,
                msg.info.origin.position.y + (iy + 0.5) * res)

    def obstacle_clearance(self, msg, x, y, search_radius):
        """Distance from a point to the nearest occupied grid-cell boundary."""
        cell = self.map_cell(msg, x, y)
        if cell is None:
            return None
        res = msg.info.resolution
        ix, iy = cell
        radius_cells = int(math.ceil((search_radius + res) / res))
        clearance = None
        for cy in range(max(0, iy - radius_cells), min(msg.info.height, iy + radius_cells + 1)):
            for cx in range(max(0, ix - radius_cells), min(msg.info.width, ix + radius_cells + 1)):
                if msg.data[cy * msg.info.width + cx] < 50:
                    continue
                center_x, center_y = self.cell_center(msg, cx, cy)
                dx = max(abs(x - center_x) - 0.5 * res, 0.0)
                dy = max(abs(y - center_y) - 0.5 * res, 0.0)
                distance = math.hypot(dx, dy)
                if clearance is None or distance < clearance:
                    clearance = distance
        return clearance

    def projected_map_diagnostic(self, x, y):
        msg = self.projected_map
        if msg is None or msg.info.resolution <= 0.0:
            return None, None
        cell = self.map_cell(msg, x, y)
        if cell is None:
            return None, None
        ix, iy = cell
        value = int(msg.data[iy * msg.info.width + ix])
        clearance = self.obstacle_clearance(msg, x, y, 0.75)
        return value, clearance

    def target_is_safe(self, msg, x, y):
        cell = self.map_cell(msg, x, y)
        if cell is None:
            return False
        ix, iy = cell
        value = int(msg.data[iy * msg.info.width + ix])
        if value < 0 or value >= 50:
            return False
        clearance = self.obstacle_clearance(
            msg, x, y, self.target_min_clearance)
        return clearance is None or clearance > self.target_min_clearance + 1e-6

    def target_is_far_enough(self, x, y):
        if self.robot_xy is None:
            return False
        return math.hypot(x - self.robot_xy[0], y - self.robot_xy[1]) \
            > self.target_min_robot_distance

    def safe_target(self, requested):
        """Keep a safe target or move it to the nearest safe known-free cell."""
        msg = self.projected_map
        if msg is None or msg.info.resolution <= 0.0:
            return None
        if self.target_is_safe(msg, requested[0], requested[1]) \
                and self.target_is_far_enough(requested[0], requested[1]):
            return requested
        cell = self.map_cell(msg, requested[0], requested[1])
        if cell is None:
            return None
        ix, iy = cell
        radius_cells = int(math.ceil(self.target_search_radius / msg.info.resolution))
        candidates = []
        for cy in range(max(0, iy - radius_cells), min(msg.info.height, iy + radius_cells + 1)):
            for cx in range(max(0, ix - radius_cells), min(msg.info.width, ix + radius_cells + 1)):
                x, y = self.cell_center(msg, cx, cy)
                distance = math.hypot(x - requested[0], y - requested[1])
                if distance > self.target_search_radius:
                    continue
                if not self.target_is_far_enough(x, y):
                    continue
                if self.target_is_safe(msg, x, y):
                    clearance = self.obstacle_clearance(
                        msg, x, y, self.target_min_clearance + msg.info.resolution)
                    candidates.append((distance,
                                       -(clearance if clearance is not None else 1e9),
                                       x, y))
        if not candidates:
            return None
        _, _, x, y = min(candidates)
        return x, y

    def pause_cb(self, msg):
        self.paused = msg.data
        if self.paused:
            self.last_sent = None
            self.valid_pub.publish(Bool(False))

    def reset_cb(self, msg):
        if msg.data:
            self.last_sent = None
            self.valid_pub.publish(Bool(False))

    def waypoint_cb(self, msg):
        if self.paused or self.robot_xy is None:
            return  # 未收到里程计前不发：SCAN 需要真实起点
        requested = (msg.point.x, msg.point.y)
        wp = self.safe_target(requested)
        if wp is None:
            self.valid_pub.publish(Bool(False))
            self.publish_diag('REJECT_UNSAFE', requested_x=requested[0],
                              requested_y=requested[1],
                              required_clearance=self.target_min_clearance,
                              search_radius=self.target_search_radius)
            rospy.logwarn_throttle(
                1.0, '[ariadne_goal_bridge] reject unsafe target (%.2f, %.2f): '
                'no known-free point with >%.2fm clearance within %.2fm',
                requested[0], requested[1], self.target_min_clearance,
                self.target_search_radius)
            return
        if self.last_sent is not None:
            d = ((wp[0] - self.last_sent[0]) ** 2 +
                 (wp[1] - self.last_sent[1]) ** 2) ** 0.5
            if d <= self.repub_dist:
                return  # 同目标去重
        now = rospy.Time.now()
        path = Path()
        path.header.stamp = now
        path.header.frame_id = 'world'
        for x, y in (self.robot_xy, wp):
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = 'world'
            ps.pose.position.x = x
            ps.pose.position.y = y
            ground_z = self.ground_z_at(x, y)
            ps.pose.position.z = self.floor_z_ref if ground_z is None else ground_z
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)
        self.last_sent = wp
        self.valid_pub.publish(Bool(True))
        map_value, clearance_2d = self.projected_map_diagnostic(wp[0], wp[1])
        self.publish_diag('FORWARDED', requested_x=requested[0], requested_y=requested[1],
                          target_x=wp[0], target_y=wp[1],
                          clearance=clearance_2d, retry=0)
        rospy.loginfo(
            '[ariadne_goal_bridge] 转发航点 requested=(%.2f, %.2f) '
            'effective=(%.2f, %.2f) ground_z=%.3f '
            'projected_occ=%s projected_clearance=%s',
            requested[0], requested[1], wp[0], wp[1],
            self.ground_z_at(wp[0], wp[1]),
            'NA' if map_value is None else str(map_value),
            'NA' if clearance_2d is None else '%.2f' % clearance_2d)


class FeedbackWaypointBridge(WaypointBridge):
    """One decision at a time. Timer owns transitions; callbacks only enqueue."""

    def __init__(self):
        from scan_planner.msg import GoalFeedback
        from geometry_msgs.msg import Twist
        from std_msgs.msg import String, Time
        self.Feedback = GoalFeedback
        self.TimeMsg = Time
        self.events = deque()
        self.decision = self.attempt = self.result = None
        self.failed = []
        self.current = None
        self.command = (0.0, 0.0, 0.0)
        self.gate = (0.0, '')
        self.odom_received = 0.0
        self.odom_stamp = None
        self.last_good = time.monotonic()
        self.valid_intervals = deque()
        self.last_tick = self.last_good
        self.last_stamp_ns = 0
        self.counts = dict(timeouts=0, retry_sent=0, recovered=0, exhausted=0)
        self.pending_retry = False
        self.last_feedback_received = 0.0
        super().__init__()
        self.timeout = float(rospy.get_param('~no_command_timeout', 6.0))
        self.motion = bool(rospy.get_param('~motion_enabled', False))
        self.result_pub = rospy.Publisher('/ariadne/goal_feedback', GoalFeedback, queue_size=10)
        self.cancel_pub = rospy.Publisher('/scan/cancel_goal', Time, queue_size=10)
        rospy.Subscriber('/scan/goal_feedback', GoalFeedback,
                         lambda m: self.events.append(('feedback', m)), queue_size=20)
        rospy.Subscriber('/cmd_vel', Twist, self.command_cb, queue_size=10)
        rospy.Subscriber('/cmd_vel_gate/status', String,
                         lambda m: setattr(self, 'gate', (time.monotonic(), m.data)), queue_size=2)
        rospy.Timer(rospy.Duration(0.1), self.tick)
        rospy.Timer(rospy.Duration(30), lambda _: self.summary())
        rospy.on_shutdown(self.summary)

    def summary(self):
        rospy.loginfo('[AR_RECOVERY_SUMMARY] %s', self.counts)

    def odom_cb(self, msg):
        if not all(math.isfinite(v) for v in (msg.pose.pose.position.x, msg.pose.pose.position.y)):
            return
        if self.odom_stamp is not None and msg.header.stamp <= self.odom_stamp:
            return
        self.odom_stamp = msg.header.stamp
        super().odom_cb(msg)
        self.odom_received = time.monotonic()

    def command_cb(self, msg):
        self.command = (time.monotonic(), math.hypot(msg.linear.x, msg.linear.y), abs(msg.angular.z))

    def waypoint_cb(self, msg):
        self.events.append(('request', msg))

    def pause_cb(self, msg):
        self.events.append(('pause', msg.data))

    def reset_cb(self, msg):
        if msg.data: self.events.append(('reset', True))

    def cancel(self):
        if self.attempt is not None:
            self.cancel_pub.publish(self.TimeMsg(self.attempt.header.stamp))

    def finish(self, state, reason):
        if self.decision is None: return
        result = self.Feedback()
        result.request_header = copy.deepcopy(self.decision.header)
        result.state, result.reason = state, reason
        self.result = result
        self.result_pub.publish(result)
        self.decision = self.attempt = None
        self.pending_retry = False
        self.valid_pub.publish(Bool(False))

    def choose(self):
        import numpy as np
        from scipy.ndimage import label
        msg = self.projected_map
        if msg is None or self.robot_xy is None or msg.info.resolution <= 0: return None
        grid = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
        cell = self.map_cell(msg, *self.robot_xy)
        if cell is None: return None
        regions, _ = label(grid == 0, np.ones((3, 3)))
        region = regions[cell[1], cell[0]]
        if region == 0: return None
        ys, xs = np.nonzero(regions == region)
        x = msg.info.origin.position.x + (xs + .5) * msg.info.resolution
        y = msg.info.origin.position.y + (ys + .5) * msg.info.resolution
        requested = (self.decision.point.x, self.decision.point.y)
        distances = np.hypot(x - requested[0], y - requested[1])
        keep = distances <= self.target_search_radius
        keep &= np.hypot(x-self.robot_xy[0], y-self.robot_xy[1]) \
            > self.target_min_robot_distance
        for fx, fy in self.failed: keep &= np.hypot(x-fx, y-fy) >= 0.6
        indices = np.flatnonzero(keep)
        for i in indices[np.argsort(distances[indices])]:
            candidate = (float(x[i]), float(y[i]))
            if self.target_is_safe(msg, *candidate) and self.target_is_safe(self.projected_map, *candidate):
                return candidate
        return None

    def send_attempt(self):
        if self.path_pub.get_num_connections() == 0:
            rospy.logwarn_throttle(5, '[AR_RECOVERY] waiting for SCAN path subscriber')
            return
        target = self.choose()
        if target is None:
            self.publish_diag('NO_CANDIDATE', requested_x=self.decision.point.x,
                              requested_y=self.decision.point.y, failed=len(self.failed))
            self.cancel()
            self.counts['exhausted'] += 1
            rospy.logwarn('[AR_RECOVERY] exhausted attempts=%d counts=%s', len(self.failed), self.counts)
            self.finish(self.Feedback.FAILED, 'NO_CANDIDATE')
            return
        path = Path()
        ns = max(rospy.Time.now().to_nsec(), self.last_stamp_ns + 1)
        self.last_stamp_ns = ns
        path.header.stamp = rospy.Time(ns // 1000000000, ns % 1000000000)
        path.header.frame_id = 'world'
        for x, y in (self.robot_xy, target):
            pose = PoseStamped()
            pose.header = copy.deepcopy(path.header)
            pose.pose.position.x, pose.pose.position.y = x, y
            ground_z = self.ground_z_at(x, y)
            pose.pose.position.z = self.floor_z_ref if ground_z is None else ground_z
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.attempt, self.current = path, target
        self.pending_retry = False
        self.last_good = time.monotonic()
        self.last_feedback_received = self.last_good
        self.valid_intervals.clear()
        if self.failed: self.counts['retry_sent'] += 1
        self.path_pub.publish(path)
        self.valid_pub.publish(Bool(True))
        map_value, clearance_2d = self.projected_map_diagnostic(target[0], target[1])
        self.publish_diag('FORWARDED', requested_x=self.decision.point.x,
                          requested_y=self.decision.point.y, target_x=target[0],
                          target_y=target[1], clearance=clearance_2d,
                          retry=len(self.failed))
        rospy.loginfo('[AR_RECOVERY] goal_sent decision=%s attempt=%s target=%s retries=%d counts=%s',
                      self.decision.header.stamp, path.header.stamp, target, len(self.failed), self.counts)

    def tick(self, _):
        now = time.monotonic()
        dt = min(now - self.last_tick, 0.2)
        self.last_tick = now
        while self.events:
            kind, msg = self.events.popleft()
            if kind in ('pause', 'reset'):
                self.cancel()
                self.finish(self.Feedback.FAILED, 'PAUSED')
                self.paused = msg if kind == 'pause' else False
                continue
            if kind == 'request':
                if self.decision is not None or self.paused: continue
                if self.result and msg.header.stamp == self.result.request_header.stamp: continue
                self.decision, self.result = msg, None
                self.failed = []
                self.pending_retry = True
            elif kind == 'feedback' and self.attempt is not None:
                if msg.request_header.stamp != self.attempt.header.stamp: continue
                self.last_feedback_received = now
                if msg.state == self.Feedback.SUCCEEDED:
                    if self.failed: self.counts['recovered'] += 1
                    self.finish(msg.state, msg.reason)
                elif msg.state == self.Feedback.FAILED:
                    self.failed.append(self.current)
                    self.attempt = None
                    self.pending_retry = True
        if self.result is not None: self.result_pub.publish(self.result)
        if self.decision is None or self.paused: return
        if not rospy.get_param('/ariadne/execution_ready', False):
            self.last_good = now
            return
        if self.pending_retry:
            if self.motion and self.failed and (now-self.gate[0] > .5 or self.gate[1] not in ('READY', 'WAIT_TRAJECTORY') or now-self.odom_received > .5):
                rospy.logwarn_throttle(5, '[AR_RECOVERY] retry pending: existing gate not ready')
                return
            if self.projected_map is not None and self.robot_xy is not None:
                self.send_attempt()
            return
        if not self.motion or self.attempt is None: return
        ct, v, w = self.command
        gt, state = self.gate
        if now-self.odom_received > .5 or now-gt > .5 or state != 'READY' or now-ct > .25:
            self.last_good = now
            self.valid_intervals.clear()
            rospy.logwarn_throttle(5, '[AR_RECOVERY] suspended: gate/odom/command not ready (%s)', state)
            return
        if v > .03 or w > .05: self.valid_intervals.append((now, dt, ct))
        while self.valid_intervals and now-self.valid_intervals[0][0] > 1.0:
            self.valid_intervals.popleft()
        if sum(item[1] for item in self.valid_intervals) >= .2:
            self.last_good = max(self.last_good, self.valid_intervals[-1][2])
        if now-self.last_good >= self.timeout:
            self.counts['timeouts'] += 1
            feedback_age = now-self.last_feedback_received
            rospy.logwarn('[AR_RECOVERY] trigger reason=NO_VALID_CMD elapsed=%.2f '
                          'feedback_age=%.2f old=%s counts=%s',
                          now-self.last_good, feedback_age, self.current, self.counts)
            self.cancel()
            self.failed.append(self.current)
            self.attempt = None
            self.pending_retry = True


if __name__ == '__main__':
    rospy.init_node('ariadne_goal_bridge')
    if rospy.get_param('~goal_feedback', False):
        FeedbackWaypointBridge()
    else:
        WaypointBridge()
    rospy.spin()
