#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compact live diagnostics for AR -> bridge -> SCAN -> motion."""
import json
import math
import time
import rospy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry, Path
from scan_planner.msg import Bspline, GoalFeedback
from std_msgs.msg import String


def key(stamp):
    return stamp.to_nsec()


class Monitor:
    def __init__(self):
        self.robot = None
        self.raw_id = None
        self.raw_target = None
        self.raw_time = 0.0
        self.path_time = 0.0
        self.scan_id = None
        self.scan_state = None
        self.distance = 0.0
        self.traj = -1
        self.active_since = 0.0
        self.progress_distance = None
        self.progress_time = 0.0
        self.stall_print = 0.0
        self.cmd = (0.0, 0.0, 0.0)
        self.gate = "UNKNOWN"
        self.ar_reason = None
        self.ar_reason_time = 0.0
        self.decision_signature = None
        self.decision_time = 0.0
        self.ar_status = None
        rospy.Subscriber("/LIO/odom_vehicle", Odometry, self.odom_cb, queue_size=2)
        rospy.Subscriber("/way_point", PointStamped, self.ar_target_cb, queue_size=5)
        rospy.Subscriber("/initial_path", Path, self.path_cb, queue_size=5)
        rospy.Subscriber("/scan/goal_feedback", GoalFeedback, self.scan_cb, queue_size=20)
        rospy.Subscriber("/planning/bspline", Bspline, self.traj_cb, queue_size=5)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_cb, queue_size=5)
        rospy.Subscriber("/cmd_vel_gate/status", String, self.gate_cb, queue_size=5)
        rospy.Subscriber("/ariadne/status", String, self.ar_status_cb, queue_size=5)
        rospy.Subscriber("/ariadne/diagnostic", String, self.ar_diag_cb, queue_size=10)
        rospy.Subscriber("/ariadne/bridge/diagnostic", String, self.bridge_diag_cb, queue_size=10)
        rospy.Timer(rospy.Duration(1.0), self.timer_cb)
        self.out("[诊断] 导航监视器已启动：AR目标 -> 目标桥 -> SCAN -> 控制器")

    @staticmethod
    def out(text):
        print(time.strftime("%H:%M:%S") + " " + text, flush=True)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.robot = (p.x, p.y, p.z)

    def ar_target_cb(self, msg):
        request_id = key(msg.header.stamp)
        if request_id == self.raw_id:
            return
        self.raw_id = request_id
        self.ar_reason = None  # 新目标后，同一等待原因若再次出现也要重新显示
        self.raw_target = (msg.point.x, msg.point.y, msg.point.z)
        self.raw_time = time.monotonic()
        distance = "NA"
        if self.robot is not None:
            distance = "%.2fm" % math.hypot(msg.point.x-self.robot[0], msg.point.y-self.robot[1])
        self.out("[AR目标] id=%d target=(%.2f, %.2f, %.2f) distance=%s" %
                 (request_id, msg.point.x, msg.point.y, msg.point.z, distance))

    def path_cb(self, msg):
        if len(msg.poses) < 2:
            self.out("[目标桥] 拒绝：initial_path点数不足")
            return
        start = msg.poses[0].pose.position
        goal = msg.poses[-1].pose.position
        self.path_time = time.monotonic()
        self.out("[目标桥] 已交给SCAN attempt=%d goal=(%.2f, %.2f, %.2f) path=%.2fm" %
                 (key(msg.header.stamp), goal.x, goal.y, goal.z,
                  math.hypot(goal.x-start.x, goal.y-start.y)))

    def scan_cb(self, msg):
        request_id = key(msg.request_header.stamp)
        changed = request_id != self.scan_id or msg.state != self.scan_state
        self.scan_id = request_id
        self.scan_state = msg.state
        self.distance = msg.distance_xy
        self.traj = msg.traj_id
        now = time.monotonic()
        if msg.state == GoalFeedback.ACTIVE:
            if changed:
                self.active_since = now
                self.progress_distance = msg.distance_xy
                self.progress_time = now
                p = msg.effective_goal
                self.out("[SCAN] 已接收目标 id=%d goal=(%.2f, %.2f, %.2f) distance=%.2fm" %
                         (request_id, p.x, p.y, p.z, msg.distance_xy))
            elif self.progress_distance is None or msg.distance_xy < self.progress_distance-0.05:
                self.progress_distance = msg.distance_xy
                self.progress_time = now
        elif changed:
            label = "到达" if msg.state == GoalFeedback.SUCCEEDED else "失败"
            self.out("[SCAN结果] %s id=%d reason=%s distance=%.3fm traj=%d" %
                     (label, request_id, msg.reason, msg.distance_xy, msg.traj_id))

    def traj_cb(self, msg):
        self.traj = msg.traj_id

    def cmd_cb(self, msg):
        self.cmd = (math.hypot(msg.linear.x, msg.linear.y), msg.angular.z, time.monotonic())

    def gate_cb(self, msg):
        if msg.data != self.gate:
            previous = self.gate
            self.gate = msg.data
            if msg.data != "READY" or previous not in ("UNKNOWN", "READY"):
                self.out("[安全门] %s -> %s" % (previous, msg.data))

    def ar_status_cb(self, msg):
        if msg.data != self.ar_status:
            self.ar_status = msg.data
            self.out("[AR状态] %s" % msg.data)

    def ar_diag_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            self.out("[AR等待] " + msg.data)
            return
        reason = data.get("reason", "UNKNOWN")
        now = time.monotonic()
        if data.get("event") == "FRONTIER_DECISION":
            signature = (reason, str(data.get("target")), data.get("recovery"))
            if signature == self.decision_signature and now-self.decision_time < 5.0:
                return
            self.decision_signature, self.decision_time = signature, now
            labels = {
                "idle": "初次选择", "hold_region": "保持当前区域",
                "region_exhausted_or_unreachable": "原区域耗尽或不可达",
                "region_two_failures": "连续失败2次，释放区域",
                "region_no_progress": "45秒无进展，释放区域",
                "select_region": "选择区域", "policy": "采用策略目标",
                "graph_route": "沿完整图推进",
                "frontiers_no_reachable_approach": "没有可达接近点，继续重试",
                "no_visible_waypoint": "没有可直达的中间航点",
                "no_frontiers": "没有剩余前沿",
            }
            description = " → ".join(labels.get(part, part) for part in reason.split(":"))
            self.out("[AR决策] %s %s frontiers=%s region=%s search=%.1fms target=%s" %
                     ("恢复搜索" if data.get("recovery") else "正常选点", description,
                      data.get("frontiers", "?"), data.get("region_frontiers", "?"),
                      float(data.get("search_ms", 0)), data.get("target")))
            return
        if reason == self.ar_reason and now-self.ar_reason_time < 5.0:
            return
        self.ar_reason, self.ar_reason_time = reason, now
        explain = {
            "frontiers_no_reachable_approach": "全局仍有前沿，当前无可达接近点；AR继续重试",
            "utility_zero_with_frontiers_no_path": "仍有前沿，但图中没有可达前沿路径",
            "utility_zero_waiting_map_growth": "没有正效用节点，等待地图增长",
            "no_valid_policy_action": "策略没有选出有效动作",
            "waypoint_update_throttle": "已有目标正在执行",
            "oscillation_break": "检测到目标往返振荡",
            "exploration_completed": "探索完成",
            "paused": "AR已暂停",
        }
        self.out("[AR等待] reason=%s（%s）frontiers=%s util_pos=%s valid_actions=%s blocked=%s stalled=%.1fs" %
                 (reason, explain.get(reason, "本轮未发布新目标"),
                  data.get("frontiers", "?"), data.get("util_pos", "?"),
                  data.get("valid_actions", "?"), data.get("blocked", "?"),
                  float(data.get("stalled", 0.0))))

    def bridge_diag_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            self.out("[目标桥] " + msg.data)
            return
        event = data.get("event")
        if event == "NO_CANDIDATE":
            self.out("[目标桥] 拒绝：AR目标附近没有满足距离和净空条件的候选点 requested=(%.2f, %.2f)" %
                     (data.get("requested_x", 0.0), data.get("requested_y", 0.0)))
        elif event == "REJECT_UNSAFE":
            self.out("[目标桥] 拒绝：目标附近无安全点 requested=(%.2f, %.2f) clearance>%.2fm" %
                     (data.get("requested_x", 0.0), data.get("requested_y", 0.0),
                      data.get("required_clearance", 0.0)))
        elif event == "FORWARDED":
            clearance = data.get("clearance")
            clearance_text = "NA" if clearance is None else "%.2fm" % float(clearance)
            self.out("[目标桥] 安全目标=(%.2f, %.2f) requested=(%.2f, %.2f) clearance=%s retry=%s" %
                     (data.get("target_x", 0.0), data.get("target_y", 0.0),
                      data.get("requested_x", 0.0), data.get("requested_y", 0.0),
                      clearance_text, data.get("retry", 0)))

    def timer_cb(self, _):
        now = time.monotonic()
        if self.scan_state == GoalFeedback.ACTIVE:
            self.out("[执行中] distance=%.2fm traj=%d cmd_linear=%.2f cmd_yaw=%.2f gate=%s goal_age=%.1fs" %
                     (self.distance, self.traj, self.cmd[0], self.cmd[1],
                      self.gate, now-self.active_since))
            if now-self.progress_time >= 4.0 and now-self.stall_print >= 4.0:
                self.stall_print = now
                command_age = now-self.cmd[2]
                if self.gate != "READY":
                    self.out("[卡住判断] 安全门未放行：" + self.gate)
                elif command_age > 0.5:
                    self.out("[卡住判断] 控制器命令超时，SCAN目标仍为ACTIVE")
                elif self.cmd[0] < 0.03 and abs(self.cmd[1]) < 0.05:
                    self.out("[卡住判断] SCAN目标仍为ACTIVE，但闭环控制器输出零速度")
                elif self.cmd[0] < 0.03:
                    self.out("[转向中] 当前主要原地转向，XY距离暂未下降")
                else:
                    self.out("[卡住判断] 控制命令非零，但目标XY距离连续4秒未明显下降")
        elif self.raw_time > self.path_time and now-self.raw_time >= 2.0:
            self.out("[卡住判断] AR已经发目标，但目标桥2秒内没有交给SCAN")
            self.path_time = self.raw_time


if __name__ == "__main__":
    rospy.init_node("navigation_status_monitor")
    Monitor()
    rospy.spin()
