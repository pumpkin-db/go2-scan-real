#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ARiADNE 航点 → SCAN-Planner 参考路径桥（A2 架构唯一自研胶水）。

官方契约：rl_planner 输出 /way_point(PointStamped, ≤2.5Hz)，由下游 waypoint
follower 导航。本机用 SCAN-Planner navi_mode=3 替代 follower：
每条 /initial_path(nav_msgs/Path) 触发一次全局参考重建，走完一段进 WAIT_TARGET。
由此推出四条接口纪律（源码依据见 scan_replan_fsm.cpp pathCallback）：
  1. Path 只需两点 [当前位置, 航点]——SCAN 对稀疏路径自动插中间点
  2. poses[0] 必须是轨迹起点（SCAN 把首点当 trajectory start）
  3. z 发当前楼层地面 z_ref（SCAN 内部自动 +body_height）
  4. 必须去重：way_point 以固定频率重复发布同一目标，
     不去重会反复触发 SCAN 整体重规划
frame_id 用 world（SCAN 全局系；数值与 map 恒等，由 map→world 静态 TF 桥保证）。
"""

import math
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Bool, Float64


class WaypointBridge(object):
    def __init__(self):
        self.repub_dist = float(rospy.get_param('~repub_dist', 0.5))
        self.target_min_clearance = float(rospy.get_param('~target_min_clearance', 0.25))
        self.target_search_radius = float(rospy.get_param('~target_search_radius', 3.0))
        if self.target_min_clearance < 0.0:
            raise ValueError('target_min_clearance must be non-negative')
        if self.target_search_radius < self.target_min_clearance:
            raise ValueError('target_search_radius must be >= target_min_clearance')
        self.robot_xy = None
        # SCAN adds its configured body_height to every /initial_path pose.
        # Therefore Path.z must be current floor ground z, not always zero.
        # Single-floor real runs do not launch Floor Context.  Allow the
        # launcher to seed the actual ground height instead of silently
        # assuming that the LIO origin is on the ground.
        self.floor_z_ref = float(rospy.get_param('~floor_z_ref', 0.0))
        self.last_sent = None
        self.projected_map = None
        self.paused = False
        # latch 不开：latch 会让迟连接的订阅者收到旧 Path 触发意外重规划
        self.path_pub = rospy.Publisher('/initial_path', Path, queue_size=1)
        self.valid_pub = rospy.Publisher('/ariadne/bridge/target_valid', Bool,
                                         queue_size=1, latch=True)
        rospy.Subscriber('/quad_0/body_pose', Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber('/floor_context/z_ref', Float64, self.floor_z_cb, queue_size=1)
        rospy.Subscriber('/way_point', PointStamped, self.waypoint_cb, queue_size=1)
        rospy.Subscriber('/projected_map', OccupancyGrid, self.map_cb, queue_size=1)
        rospy.Subscriber('/ariadne/lifecycle/pause', Bool, self.pause_cb, queue_size=1)
        rospy.Subscriber('/ariadne/lifecycle/reset_for_floor', Bool, self.reset_cb, queue_size=1)
        self.valid_pub.publish(Bool(False))
        rospy.loginfo('[ariadne_goal_bridge] floor_z_ref=%.3f repub=%.2fm '
                      'target_clearance>%.2fm search=%.2fm',
                      self.floor_z_ref, self.repub_dist,
                      self.target_min_clearance, self.target_search_radius)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.robot_xy = (p.x, p.y)

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

    def safe_target(self, requested):
        """Keep a safe target or move it to the nearest safe known-free cell."""
        msg = self.projected_map
        if msg is None or msg.info.resolution <= 0.0:
            return None
        if self.target_is_safe(msg, requested[0], requested[1]):
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
            ps.pose.position.z = self.floor_z_ref
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)
        self.last_sent = wp
        self.valid_pub.publish(Bool(True))
        map_value, clearance = self.projected_map_diagnostic(wp[0], wp[1])
        rospy.loginfo(
            '[ariadne_goal_bridge] 转发航点 requested=(%.2f, %.2f) '
            'effective=(%.2f, %.2f) z_ref=%.3f '
            'projected_occ=%s projected_clearance=%s',
            requested[0], requested[1], wp[0], wp[1], self.floor_z_ref,
            'NA' if map_value is None else str(map_value),
            'NA' if clearance is None else '%.2f' % clearance)


if __name__ == '__main__':
    rospy.init_node('ariadne_goal_bridge')
    WaypointBridge()
    rospy.spin()
