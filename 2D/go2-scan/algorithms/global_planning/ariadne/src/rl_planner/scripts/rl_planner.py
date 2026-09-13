#!/usr/bin/env python
# -*- coding: utf-8 -*-
import warnings
warnings.simplefilter("ignore", UserWarning)

import rospy
import rospkg
import numpy as np
import torch
import os
import time
import json
from collections import deque
from std_msgs.msg import Bool, Float32, Float64, Header, Int32, String
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from agent import Agent
from exploration_continuity import FrontierContinuity
from model import PolicyNet
from node_manager import NodeManager
from utils import *
import parameter


class Runner:
    def __init__(self):
        self.map_info = None
        self.device = 'cpu'
        self.step = 0
        self.paused = False
        self.reset_requested = False
        self.session_id = 1
        self.goal_feedback_enabled = rospy.get_param('~goal_feedback', False)
        self.pending_goal = None
        self.goal_results = deque()
        self.failed_until = {}
        self.goal_stamp_ns = 0
        self.continuity = FrontierContinuity()
        if self.goal_feedback_enabled:
            from scan_planner.msg import GoalFeedback
            rospy.Subscriber('/ariadne/goal_feedback', GoalFeedback,
                             lambda m: self.goal_results.append(m), queue_size=20)

        # visualization
        self.publish_graph = rospy.get_param('~publish_graph', True)

        # map related
        parameter.CELL_SIZE = rospy.get_param('~map_resolution', parameter.CELL_SIZE)
        parameter.FREE = rospy.get_param('~map_free_value', parameter.FREE)
        parameter.OCCUPIED = rospy.get_param('~map_occupied_value', parameter.OCCUPIED)
        parameter.UNKNOWN = rospy.get_param('~map_unknown_value', parameter.UNKNOWN)

        # utility related
        parameter.SENSOR_RANGE = rospy.get_param('~sensor_range', parameter.SENSOR_RANGE)
        parameter.UTILITY_RANGE = rospy.get_param('~utility_range_factor', 0.5) * parameter.SENSOR_RANGE
        parameter.MIN_UTILITY = rospy.get_param('~min_utility', parameter.MIN_UTILITY)
        parameter.FRONTIER_CELL_SIZE = rospy.get_param('~frontier_downsample_factor', 1) * parameter.CELL_SIZE

        # graph related
        parameter.NODE_RESOLUTION = rospy.get_param('~node_resolution', parameter.NODE_RESOLUTION)
        parameter.CLUSTER_RANGE = rospy.get_param('~frontier_cluster_range', parameter.CLUSTER_RANGE)
        parameter.THR_NEXT_WAYPOINT = rospy.get_param('~next_waypoint_threshold', parameter.THR_NEXT_WAYPOINT)
        parameter.THR_GRAPH_HARD_UPDATE = rospy.get_param('~hard_update_threshold', parameter.THR_GRAPH_HARD_UPDATE)

        # replanning related
        parameter.THR_TO_WAYPOINT = rospy.get_param('~waypoint_threshold', parameter.THR_TO_WAYPOINT)
        parameter.AVOID_OSCILLATION = rospy.get_param('~avoid_waypoint_oscillation', parameter.AVOID_OSCILLATION)
        parameter.ENABLE_SAVE_MODE = rospy.get_param('~enable_save_mode', parameter.ENABLE_SAVE_MODE)
        parameter.ENABLE_DSTARLITE = rospy.get_param('~enable_dstarlite', parameter.ENABLE_DSTARLITE)
        self.enable_escape_recovery = rospy.get_param('~enable_escape_recovery', False)
        self.escape_min_distance = rospy.get_param(
            '~escape_min_distance', 2 * parameter.NODE_RESOLUTION)
        self.escape_required_displacement = rospy.get_param(
            '~escape_required_displacement', 3 * parameter.NODE_RESOLUTION)
        self.escape_required_map_growth = rospy.get_param(
            '~escape_required_map_growth', 20)
        frequency = rospy.get_param('~replanning_frequency', 2.5)
        self.waypoint_update_min_interval = rospy.get_param(
            '~waypoint_update_min_interval', 0.0)

        # network model file
        self.model_file = "checkpoint.pth"

        # robot coordination wrt map frame
        self.robot_location = None
        # All AR visualization geometry is drawn on the current floor plane.
        # Fixed mode keeps the launch value; elevation mode publishes this value.
        self.ground_z_ref = float(rospy.get_param('~ground_z_ref', 0.0))

        # the grid occupied by the robot
        self.robot_cell = None

        # initialize robot planner
        self.robot = None
        self.init_agent()
        self.start = None

        # waypoint
        self.next_waypoint_list = []
        self.history_waypoint_list = []
        self.next_waypoint = None
        self._last_waypoint_publish_time = 0.0

        # termination status
        self.done = False

        # 停滞式完成判定状态（2026-08-25）：地图签名与最后变化时刻
        self._last_map_signature = None
        self._last_map_change_time = time.time()
        self.stalled_complete_seconds = rospy.get_param('~stalled_complete_seconds', 20.0)

        # save mode
        self.save_mode = False
        self.escape_mode = False
        self.escape_arrived = False
        self.escape_origin = None
        self.escape_start_known_cells = 0
        self.escape_excluded_nodes = set()
        self.policy_blocked_nodes = set()
        # STOP_REASON 诊断（2026-08-28，纯日志不改变行为）
        self._stop_state = {'reason': None, 't': 0.0}
        self._done_reason = ''

        # subscribers
        map_topic = rospy.get_param('~map_topic', '/projected_map')
        rospy.Subscriber(map_topic, OccupancyGrid, self.get_map_callback, queue_size=1)
        rospy.Subscriber('/state_estimation', Odometry, self.get_loc_callback, queue_size=1)
        rospy.Subscriber('/floor_context/z_ref', Float64,
                         self.floor_z_callback, queue_size=1)
        rospy.Subscriber('/scan/effective_goal', Path,
                         self.effective_goal_callback, queue_size=5)

        # publishers
        self.waypoint_pub = rospy.Publisher('/way_point', PointStamped, queue_size=1)
        self.run_time_pub = rospy.Publisher('/runtime', Float32, queue_size=1)
        self.edge_pub = rospy.Publisher('/edge', Marker, queue_size=1)
        self.node_pub = rospy.Publisher('/node', PointCloud2, queue_size=1)
        self.frontier_pub = rospy.Publisher('/frontier', PointCloud2, queue_size=1)
        self.status_pub = rospy.Publisher('/ariadne/status', String, queue_size=1, latch=True)
        self.diagnostic_pub = rospy.Publisher('/ariadne/diagnostic', String, queue_size=5)
        self.session_pub = rospy.Publisher('/ariadne/session_id', Int32, queue_size=1, latch=True)
        self.target_valid_pub = rospy.Publisher('/ariadne/target_valid', Bool, queue_size=1, latch=True)
        rospy.Subscriber('/ariadne/lifecycle/pause', Bool, self.pause_callback, queue_size=1)
        rospy.Subscriber('/ariadne/lifecycle/reset_for_floor', Bool, self.reset_callback, queue_size=1)
        self.session_pub.publish(Int32(self.session_id))
        self.target_valid_pub.publish(Bool(False))
        self.status_pub.publish(String('STARTING'))
        
        # get map and robot location
        while self.map_info is None or self.robot_location is None:
            pass

        self.publish_status()

        rate = rospy.Rate(20)
        rospy.Timer(rospy.Duration(1 / frequency), self.run)
        try:
            rate.sleep()
            rospy.spin()
        except KeyboardInterrupt:
            pass

    def get_map_callback(self, msg):
        t1 = time.time()
        delta = msg.info.resolution
        map_origin_x = msg.info.origin.position.x
        map_origin_y = msg.info.origin.position.y

        map_width = msg.info.width
        map_height = msg.info.height
        ros_map = np.array(np.array(msg.data).reshape(map_height, map_width).astype(np.int8))

        # padding the map with unknown area to avoid a frontier calculation issue
        pad_size = int(parameter.NODE_RESOLUTION // parameter.CELL_SIZE + 1)
        processed_map = np.pad(ros_map, ((pad_size, pad_size), (pad_size, pad_size)), 'constant', constant_values=parameter.UNKNOWN)
        map_origin_x -= delta * pad_size
        map_origin_y -= delta * pad_size
        robot_belief_map = processed_map

        self.map_info = MapInfo(robot_belief_map, map_origin_x, map_origin_y, delta)

        # 停滞式完成判定（第一杠杆，2026-08-25）：记录地图最后一次「增长」的时刻。
        # 已知区域格数变化才算增长；纯 utility 判完成会在门洞前沿不可见时假停。
        known_cells = int((ros_map != parameter.UNKNOWN).sum())
        sig = (map_width, map_height, known_cells)
        if sig != self._last_map_signature:
            self._last_map_signature = sig
            self._last_map_change_time = time.time()

        t2 = time.time()
        # print("process map using {}".format(t2 - t1))

    def floor_z_callback(self, msg):
        z = float(msg.data)
        if np.isfinite(z):
            self.ground_z_ref = z

    def get_loc_callback(self, msg):
        if self.map_info is None:
            return
        self.robot_location = np.around(np.array([msg.pose.pose.position.x, msg.pose.pose.position.y]), 1)
        if self.start is None:

            x = np.array([(self.robot_location[0] // parameter.NODE_RESOLUTION) * parameter.NODE_RESOLUTION, (self.robot_location[0] // parameter.NODE_RESOLUTION + 1) * parameter.NODE_RESOLUTION])
            y = np.array([(self.robot_location[1] // parameter.NODE_RESOLUTION) * parameter.NODE_RESOLUTION, (self.robot_location[1] // parameter.NODE_RESOLUTION + 1) * parameter.NODE_RESOLUTION])
            t1, t2 = np.meshgrid(x, y)
            candidate_starts = np.vstack([t1.T.ravel(), t2.T.ravel()]).T
            dis_robot = np.linalg.norm(candidate_starts - self.robot_location, axis=1)
            sorted_candidate_starts = candidate_starts[np.argsort(dis_robot)]

            for start in sorted_candidate_starts:
                if is_free(start, self.map_info):
                    self.start = start
                    break

            assert self.start is not None, rospy.logwarn("can not find valid start point")

            self.start = np.around(self.start, 1)
            self.robot.node_manager = NodeManager(self.start)
            print("initialize quad tree at", self.start)
            print("initialize robot location at", self.robot_location)
            if hasattr(self, 'status_pub'):
                self.publish_status()
        self.robot_cell = get_cell_position_from_coords(self.robot_location, self.map_info)

    def waypoint_wrapper(self, loc):
        way_point = PointStamped()
        way_point.header.frame_id = "map"
        way_point.header.stamp = rospy.Time.now()
        way_point.point.x = loc[0]
        way_point.point.y = loc[1]
        way_point.point.z = self.ground_z_ref
        return way_point

    def init_agent(self):
        policy_net = PolicyNet(parameter.NODE_INPUT_DIM, parameter.EMBEDDING_DIM).to(self.device)
        model_folder = os.path.join(rospkg.RosPack().get_path('rl_planner'), 'scripts/model')
        model_file = os.path.join(model_folder, self.model_file)
        policy_net.load_state_dict(torch.load(model_file, map_location=self.device)['policy_model'])

        self.robot = Agent(policy_net, self.device, self.publish_graph)

    def publish_status(self):
        if self.paused:
            status = 'PAUSED'
        elif self.done:
            status = 'COMPLETE'
        elif self.start is None:
            status = 'STARTING'
        else:
            status = 'EXPLORING'
        self.status_pub.publish(String(status))

    def pause_callback(self, msg):
        self.paused = msg.data
        if self.paused:
            self.next_waypoint_list = []
            self.next_waypoint = None
            self._last_waypoint_publish_time = 0.0
            self.target_valid_pub.publish(Bool(False))
        self.publish_status()

    def reset_callback(self, msg):
        if msg.data:
            self.reset_requested = True

    def reset_for_floor(self):
        """Discard all decision/session state while retaining checkpoint and latest inputs."""
        self.init_agent()
        self.pending_goal = None
        self.failed_until.clear()
        self.continuity.reset()
        self.goal_results.clear()
        self.step = 0
        self.start = None
        self.robot_cell = None
        self.next_waypoint_list = []
        self.history_waypoint_list = []
        self.next_waypoint = None
        self._last_waypoint_publish_time = 0.0
        self.done = False
        self._done_reason = ''
        self._last_map_signature = None
        self._last_map_change_time = time.time()
        self.save_mode = False
        self.escape_mode = False
        self.escape_arrived = False
        self.escape_origin = None
        self.escape_start_known_cells = 0
        self.escape_excluded_nodes = set()
        self.policy_blocked_nodes = set()
        self._stop_state = {'reason': None, 't': 0.0}
        self.session_id += 1
        self.reset_requested = False
        self.session_pub.publish(Int32(self.session_id))
        self.target_valid_pub.publish(Bool(False))
        self.publish_status()
        rospy.loginfo('[ariadne] reset for floor: session=%d', self.session_id)

    def publish_waypoint(self, waypoint):
        if self.goal_feedback_enabled:
            if self.pending_goal is not None: return
            ns = max(rospy.Time.now().to_nsec(), self.goal_stamp_ns + 1)
            self.goal_stamp_ns = ns
            waypoint.header.stamp = rospy.Time(ns // 1000000000, ns % 1000000000)
            self.pending_goal = waypoint
        self.waypoint_pub.publish(waypoint)
        self.target_valid_pub.publish(Bool(True))
        self._last_waypoint_publish_time = time.time()

    def effective_goal_callback(self, msg):
        """Accept SCAN's collision-adjusted endpoint for the current ARiADNE goal.

        The two Path poses are [requested goal, effective goal].  Matching the
        requested XY prevents delayed feedback from overwriting a newer goal.
        """
        if self.next_waypoint is None or len(msg.poses) < 2:
            return
        requested = np.array([
            msg.poses[0].pose.position.x,
            msg.poses[0].pose.position.y])
        if np.linalg.norm(requested - self.next_waypoint) > 0.25:
            return
        effective = np.array([
            msg.poses[1].pose.position.x,
            msg.poses[1].pose.position.y])
        adjustment = np.linalg.norm(effective - requested)
        self.next_waypoint = effective
        if adjustment > 0.05:
            rospy.logwarn(
                '[ariadne] SCAN adjusted active goal requested=(%.2f,%.2f) '
                'effective=(%.2f,%.2f) retreat=%.2fm',
                requested[0], requested[1], effective[0], effective[1], adjustment)

    def _log_stop(self, reason, robot_node_location=None):
        """STOP_REASON 诊断：ARiADNE 本拍不发布航点时记录原因与决策上下文。
        同原因 5s 节流；换原因立即记。纯日志，不改变任何控制流。"""
        now = time.time()
        if reason == self._stop_state['reason'] and now - self._stop_state['t'] < 5.0:
            return
        self._stop_state['reason'] = reason
        self._stop_state['t'] = now
        util_pos = -1
        valid = -1
        frontier_count = -1
        try:
            util_pos = int(sum(1 for u in self.robot.key_utility if u > 0))
        except Exception:
            pass
        try:
            frontier_count = len(self.robot.frontier)
        except Exception:
            pass
        try:
            if robot_node_location is not None:
                entry = self.robot.node_manager.nodes_dict.find(list(robot_node_location))
                if entry is not None:
                    valid = len(getattr(entry.data, 'neighbor_set', set()) or set())
        except Exception:
            valid = -2
        try:
            diagnostic = dict(reason=reason, util_pos=util_pos,
                              valid_actions=valid, frontiers=getattr(self, "_global_frontier_count", frontier_count),
                              blocked=len(self.policy_blocked_nodes),
                              stalled=now - self._last_map_change_time,
                              done_reason=self._done_reason,
                              escape_mode=self.escape_mode,
                              escape_arrived=self.escape_arrived,
                              excluded=len(self.escape_excluded_nodes))
            self.diagnostic_pub.publish(String(json.dumps(diagnostic, ensure_ascii=False)))
            rospy.loginfo(
                "STOP_REASON=%s util_pos=%d valid_actions=%d blocked=%d "
                "stalled=%.1fs stall_done=%s escape_mode=%s escape_arrived=%s excluded=%d",
                reason, util_pos, valid, len(self.policy_blocked_nodes),
                now - self._last_map_change_time, self._done_reason,
                self.escape_mode, self.escape_arrived, len(self.escape_excluded_nodes))
        except Exception:
            pass

    def update_planning_graph(self):
        """Refresh perception without selecting or replacing an execution goal."""
        self.robot.node_manager.check_valid_node(self.robot_location, self.map_info)
        robot_node_location = self.robot_location
        if not np.array_equal(self.robot_location, self.start):
            if len(self.robot.node_manager.nodes_dict) == 0:
                robot_node_location = self.start
            else:
                nearest = self.robot.node_manager.nodes_dict.nearest_neighbors(
                    self.robot_location.tolist(), 1)[0]
                robot_node_location = nearest.data.coords
        self.robot.update_planning_state(self.map_info, robot_node_location)
        return robot_node_location

    def continuous_frontier_waypoint(self, proposed, frontiers):
        """Select only after terminal SCAN feedback; never determine arrival."""
        if not self.goal_feedback_enabled:
            return proposed
        decision_started = time.perf_counter()
        map_info = self.map_info
        def free(p):
            cell = get_cell_position_from_coords(np.asarray(p), map_info)
            return (0 <= cell[0] < map_info.map.shape[1] and
                    0 <= cell[1] < map_info.map.shape[0] and
                    map_info.map[cell[1], cell[0]] == parameter.FREE)
        def clear(a, b):
            return free(a) and free(b) and not check_collision(
                np.asarray(a), np.asarray(b), map_info)
        nodes = {tuple(e.data.coords): e.data
                 for e in self.robot.node_manager.nodes_dict}
        next_location = self.continuity.choose(
            proposed, frontiers, nodes, self.robot.location,
            self.robot_location, free, clear, self.policy_blocked_nodes,
            time.monotonic(), int((map_info.map != parameter.UNKNOWN).sum()),
            observation_range=parameter.UTILITY_RANGE,
            waypoint_range=parameter.THR_NEXT_WAYPOINT + parameter.NODE_RESOLUTION)
        self.diagnostic_pub.publish(String(json.dumps(dict(
            event='FRONTIER_DECISION', reason=self.continuity.reason,
            frontiers=len(frontiers),
            region_frontiers=0 if self.continuity.active is None else len(self.continuity.active),
            recovery=proposed is None,
            target=None if next_location is None else next_location.tolist(),
            search_ms=round((time.perf_counter()-decision_started)*1000.0, 1)),
            ensure_ascii=False)))
        rospy.loginfo('[AR区域] %s frontiers=%d region=%d target=%s',
                      self.continuity.reason, len(frontiers),
                      0 if self.continuity.active is None else len(self.continuity.active),
                      None if next_location is None else next_location.tolist())
        return next_location

    def run(self, event=None):
        t1 = time.time()
        if self.goal_feedback_enabled:
            while self.goal_results:
                result = self.goal_results.popleft()
                if self.pending_goal is None or result.request_header.stamp != self.pending_goal.header.stamp:
                    continue
                if result.state not in (result.SUCCEEDED, result.FAILED): continue
                self.continuity.feedback(
                    (self.pending_goal.point.x, self.pending_goal.point.y),
                    result.state == result.SUCCEEDED, time.monotonic())
                if result.state == result.FAILED:
                    p = self.pending_goal.point
                    self.failed_until[(p.x, p.y)] = time.monotonic() + 30.0
                self.pending_goal = None
                self.next_waypoint_list = []
                self.next_waypoint = None
                self.history_waypoint_list = []
            if not self.reset_requested and (self.pending_goal is not None or
                    not rospy.get_param('/ariadne/execution_ready', False)):
                # Waiting blocks decisions only. Keep frontiers and graph current,
                # including for RViz subscribers that connect after the first goal.
                if self.start is not None:
                    self.update_planning_graph()
                    if self.publish_graph:
                        self.visualize_graph()
                if self.pending_goal is not None and not self.paused:
                    self.waypoint_pub.publish(self.pending_goal)  # same request ID: delivery retry, never a new decision
                return
            self.failed_until = {p: t for p, t in self.failed_until.items() if t > time.monotonic()}
            self.policy_blocked_nodes = set(self.failed_until)
        if self.reset_requested:
            self.reset_for_floor()
            return
        if self.paused:
            self._log_stop('paused')
            return
        if self.start is None:
            self.publish_status()
            return
        # no more planning if exploration is completed
        if self.done:
             self._log_stop('done_' + (self._done_reason or 'unknown'))
             return

        if self.escape_mode:
            if np.linalg.norm(self.next_waypoint - self.robot_location) > parameter.THR_TO_WAYPOINT:
                self._log_stop('escape_walking')
                return
            if self.next_waypoint_list:
                next_waypoint = self.next_waypoint_list.pop(0)
                while check_collision(self.robot_location, np.asarray(next_waypoint), self.map_info) is False \
                        and np.linalg.norm(self.robot_location - np.asarray(next_waypoint)) < \
                        (parameter.THR_NEXT_WAYPOINT + parameter.NODE_RESOLUTION) \
                        and self.next_waypoint_list:
                    next_waypoint = self.next_waypoint_list.pop(0)
                self.next_waypoint = np.asarray(next_waypoint)
                self.publish_waypoint(self.waypoint_wrapper(self.next_waypoint))
                return

            self.escape_mode = False
            self.escape_arrived = True

        if self.save_mode:
            if np.linalg.norm(self.next_waypoint - self.robot_location) > parameter.THR_TO_WAYPOINT:
                self._log_stop('save_mode')
                return
            else:
                if len(self.next_waypoint_list) > 0:
                    next_waypoint = self.next_waypoint_list.pop(0)
                    while check_collision(self.robot_location, np.array(next_waypoint), self.map_info) is False\
                            and np.linalg.norm(self.robot_location - np.array(next_waypoint)) < (parameter.THR_NEXT_WAYPOINT + parameter.NODE_RESOLUTION)\
                            and len(self.next_waypoint_list) > 0:
                        next_waypoint = self.next_waypoint_list.pop(0)
                    self.next_waypoint = next_waypoint

                    self.history_waypoint_list.append((self.next_waypoint[0], self.next_waypoint[1]))
                    waypoint_msg = self.waypoint_wrapper(self.next_waypoint)
                    self.publish_waypoint(waypoint_msg)
                    run_time = Float32()
                    run_time.data = time.time() - t1

                    # publish
                    self.run_time_pub.publish(run_time)
                    return
                else:
                    self.save_mode = False
                    rospy.logwarn("Switch back to RL")

        # SCAN rebuilds its full reference trajectory for every distinct goal.
        # Keep the active goal stable for a short, configurable window instead
        # of feeding it policy changes at the inference frequency.
        if self.next_waypoint is not None \
                and np.linalg.norm(self.next_waypoint - self.robot_location) > parameter.THR_TO_WAYPOINT \
                and time.time() - self._last_waypoint_publish_time < self.waypoint_update_min_interval:
            self._log_stop('waypoint_update_throttle')
            return


        # check and solve oscillation between two waypoints
        if parameter.AVOID_OSCILLATION and len(self.history_waypoint_list) > 4:
            if self.history_waypoint_list[-1] == self.history_waypoint_list[-3] and self.history_waypoint_list[-2] == self.history_waypoint_list[-4]:
                self.next_waypoint_list = []
                if np.linalg.norm(self.next_waypoint - self.robot_location) > parameter.THR_TO_WAYPOINT:
                    self._log_stop('oscillation_break')
                    return

        # if planned one more step, use it
        if len(self.next_waypoint_list) > 0:
            if np.linalg.norm(self.next_waypoint - self.robot_location) > parameter.THR_TO_WAYPOINT:
                pass
            else:
                self.robot_location = self.next_waypoint
                self.next_waypoint = self.next_waypoint_list.pop(0)
                waypoint_msg = self.waypoint_wrapper(self.next_waypoint)
                self.publish_waypoint(waypoint_msg)
        self.next_waypoint_list = []
        # print("robot location at", self.robot_location)

        robot_node_location = self.update_planning_graph()

        if self.escape_arrived:
            known_cells = int((self.map_info.map != parameter.UNKNOWN).sum())
            map_growth = known_cells - self.escape_start_known_cells
            displacement = np.linalg.norm(self.robot_location - self.escape_origin)
            if map_growth >= self.escape_required_map_growth \
                    and displacement >= self.escape_required_displacement:
                self.escape_arrived = False
                self.escape_origin = None
                self.escape_excluded_nodes = set()
                self.history_waypoint_list = []
                rospy.logwarn(
                    "Escape recovery completed: displacement=%.1fm, map_growth=%d; "
                    "switch back to RL", displacement, map_growth)
            else:
                min_origin_distance = max(
                    self.escape_required_displacement,
                    displacement + parameter.NODE_RESOLUTION)
                escape_path, escape_target, path_distance, frontier_distance = \
                    self.robot.node_manager.find_escape_path(
                        self.robot.location, self.escape_excluded_nodes,
                        self.escape_min_distance, self.escape_origin,
                        min_origin_distance)
                if escape_path:
                    self.escape_excluded_nodes.add(tuple(escape_target))
                    self.next_waypoint_list = escape_path
                    self.next_waypoint = np.asarray(self.next_waypoint_list.pop(0))
                    self.escape_mode = True
                    self.escape_arrived = False
                    rospy.logwarn(
                        "Continue escape recovery: target=(%.1f, %.1f), path=%.1fm, "
                        "frontier_offset=%.1fm, displacement=%.1fm, map_growth=%d",
                        escape_target[0], escape_target[1], path_distance,
                        frontier_distance, displacement, map_growth)
                    self.publish_waypoint(self.waypoint_wrapper(self.next_waypoint))
                    return

                self.escape_arrived = False
                self.escape_origin = None
                self.escape_excluded_nodes = set()
                self.history_waypoint_list = []
                rospy.logwarn(
                    "Escape recovery stopped: no farther frontier-backed node "
                    "(displacement=%.1fm, map_growth=%d)", displacement, map_growth)

        # A zero key-node utility is not proof that exploration is complete:
        # graph rarefaction can temporarily lose utility while real frontier
        # cells are still visible.  Reuse ARiADNE's freshly computed path to
        # the nearest reachable frontier before considering completion.
        global_frontiers = get_frontier_in_map(self.map_info)
        self._global_frontier_count = len(global_frontiers)
        if sum(self.robot.key_utility) == 0:
            frontier_count = len(global_frontiers)
            frontier_path = self.robot.node_manager.path_to_nearest_frontier
            if frontier_count > 0:
                if self.goal_feedback_enabled:
                    recovery = self.continuous_frontier_waypoint(None, global_frontiers)
                    if recovery is not None:
                        self.next_waypoint = recovery
                        self.publish_waypoint(self.waypoint_wrapper(recovery))
                        self.step += 1
                        if self.publish_graph: self.visualize_graph()
                        return
                    self._log_stop('frontiers_no_reachable_approach', robot_node_location)
                    return
                if frontier_path:
                    self.next_waypoint_list = list(frontier_path)
                    self.next_waypoint = np.asarray(self.next_waypoint_list.pop(0))
                    rospy.logwarn(
                        "Zero policy utility with %d frontiers; use nearest-frontier path",
                        frontier_count)
                    self.publish_waypoint(self.waypoint_wrapper(self.next_waypoint))
                    return
                self._log_stop('utility_zero_with_frontiers_no_path', robot_node_location)
                return

            # 停滞式完成判定（2026-08-25）：效用全零还不够，必须地图也静止满
            # STALLED_COMPLETE_SECONDS 秒才算完成。否则视为暂时停滞（门洞前沿不可见等），
            # 等地图更新后前沿自然重现。
            stalled_seconds = time.time() - self._last_map_change_time
            if stalled_seconds < self.stalled_complete_seconds:
                self._log_stop('utility_zero_waiting_map_growth', robot_node_location)
                return

            g = "\033[92m"
            n= "\033[0m"
            rospy.loginfo(f"{g}Exploration Completed{n}")
            self._done_reason = 'stalled_complete' if self.stalled_complete_seconds > 0 else 'utility_zero_immediate'
            self._log_stop('exploration_completed')
            self.done = True
            self.target_valid_pub.publish(Bool(False))
            self.publish_status()
            run_time = Float32()
            run_time.data = 0
            self.run_time_pub.publish(run_time)
            return

        # get rl observation
        t2 = time.time()
        observation = self.robot.get_observation(self.robot_location)
        t3 = time.time()

        # network inference to get next waypoint
        next_location, next_node_index = self.robot.select_next_waypoint(
            observation, excluded_positions=self.policy_blocked_nodes,
            position_penalty=lambda p: self.continuity.penalty(p, time.monotonic()))
        if next_location is None:
            # All neighboring actions may be inside the 30 s failure cooldown.
            # Release only the oldest entry so exploration can continue while
            # retaining the remaining failure history.
            if self.failed_until:
                released = min(self.failed_until, key=self.failed_until.get)
                del self.failed_until[released]
                self.policy_blocked_nodes = set(self.failed_until)
                rospy.logwarn(
                    "All policy actions blocked; release oldest failed target %s",
                    released)
                next_location, next_node_index = self.robot.select_next_waypoint(
                    observation, excluded_positions=self.policy_blocked_nodes,
                    position_penalty=lambda p: self.continuity.penalty(p, time.monotonic()))
            if next_location is None and not self.goal_feedback_enabled:
                self._log_stop('no_valid_policy_action')
                return
        if self.goal_feedback_enabled:
            next_location = self.continuous_frontier_waypoint(next_location, global_frontiers)
            if next_location is None:
                self._log_stop('frontiers_no_reachable_approach', robot_node_location)
                return
            self.next_waypoint = next_location
            self.publish_waypoint(self.waypoint_wrapper(next_location))
            self.step += 1
            if self.publish_graph: self.visualize_graph()
            return

        self.next_waypoint_list.append(next_location)
        if len(self.history_waypoint_list) > 0:
            if (next_location[0], next_location[1]) != self.history_waypoint_list[-1]:
                self.history_waypoint_list.append((next_location[0], next_location[1]))
        else:
            self.history_waypoint_list.append((next_location[0], next_location[1]))

        # planning one more step if next node's utility is zero
        if self.robot.node_manager.nodes_dict.find(next_location.tolist()).data.utility == 0:
            next_observation = self.robot.get_next_observation(next_node_index, observation)
            next_next_location, _ = self.robot.select_next_waypoint(
                next_observation, excluded_positions=self.policy_blocked_nodes)

            # if next waypoint is too close, go to the next next waypoint
            if np.linalg.norm(next_location - self.robot_location) < parameter.NODE_RESOLUTION:
                self.next_waypoint_list = []

            self.next_waypoint_list.append(next_next_location)

        t4 = time.time()
        # print("next waypoint at", next_location)
        # print("update planning state using {}".format(t2 - t1))
        # print("prepare tensor input using {}".format(t3 - t2))
        # print("neural network inference using {}".format(t4-t3))

        # if rl gets stuck, go to nearest frontier
        if parameter.ENABLE_SAVE_MODE:
            if self.detect_waypoint_loop():
                self.next_waypoint_list = self.robot.node_manager.path_to_nearest_frontier
                self.save_mode = True
                rospy.logwarn("Switch to save mode")

        if self.enable_escape_recovery and self.detect_waypoint_loop():
            loop_nodes = set(self.history_waypoint_list[-6:])
            self.policy_blocked_nodes.update(loop_nodes)
            if len(self.policy_blocked_nodes) > 32:
                self.policy_blocked_nodes = set(loop_nodes)
            self.escape_origin = self.robot_location.copy()
            self.escape_start_known_cells = int(
                (self.map_info.map != parameter.UNKNOWN).sum())
            self.escape_excluded_nodes = self.policy_blocked_nodes.copy()
            escape_path, escape_target, path_distance, frontier_distance = \
                self.robot.node_manager.find_escape_path(
                    self.robot.location, self.escape_excluded_nodes,
                    self.escape_min_distance, self.escape_origin,
                    self.escape_min_distance)
            if escape_path:
                self.escape_excluded_nodes.add(tuple(escape_target))
                self.next_waypoint_list = escape_path
                self.escape_mode = True
                rospy.logwarn(
                    "Switch to escape recovery: target=(%.1f, %.1f), path=%.1fm, "
                    "frontier_offset=%.1fm, steps=%d",
                    escape_target[0], escape_target[1], path_distance,
                    frontier_distance, len(escape_path))

        # get waypoint message
        self.next_waypoint = self.next_waypoint_list.pop(0)
        waypoint_msg = self.waypoint_wrapper(self.next_waypoint)

        # get planning time message
        run_time = Float32()
        run_time.data = t4 - t1

        # publish
        self.run_time_pub.publish(run_time)
        self.publish_waypoint(waypoint_msg)
        self._stop_state['reason'] = None  # 恢复发布：下一次停止立即记录

        self.step += 1
        if self.publish_graph:
            self.visualize_graph()

    def detect_waypoint_loop(self, max_length=6):
        if len(self.history_waypoint_list) < max_length:
            return False

        waypoint_list_to_check = self.history_waypoint_list[-max_length:]
        loop =[]
        for i, waypoint in enumerate(waypoint_list_to_check[:-1]):
            if waypoint == waypoint_list_to_check[-1]:
                loop = waypoint_list_to_check[i:]

        if loop:
            loop_length = len(loop)
            if len(self.history_waypoint_list) < 2 * loop_length + 1:
                return False
            waypoint_list_to_check2 = self.history_waypoint_list[-max_length-loop_length+1:-loop_length+1]
            # print("length check", waypoint_list_to_check2, loop)
            loop2 = []
            for i, waypoint in enumerate(waypoint_list_to_check2[:-1]):
                if waypoint == waypoint_list_to_check2[-1]:
                    loop2 = waypoint_list_to_check2[i:]
                    break
            if loop2:
                return True
            else:
                return False
        return False

    def visualize_graph(self):
        # visualize edges
        edges = Marker()
        edges.header.frame_id = 'map'
        edges.header.stamp = rospy.Time.now()
        edges.type = Marker.LINE_LIST
        edges.scale.x = 0.1
        edges.color.r = 0.0
        edges.color.g = 0.6
        edges.color.b = 0.0
        edges.color.a = 1.0
        edges.pose.orientation.x = 0.0
        edges.pose.orientation.y = 0.0
        edges.pose.orientation.z = 0.0
        edges.pose.orientation.w = 1.0

        for coords in self.robot.key_node_coords:
            node = self.robot.node_manager.key_node_dict[(coords[0], coords[1])]
            for neighbor_coords in node.neighbor_set:
                start = Point()
                start.x = coords[0]
                start.y = coords[1]
                start.z = self.ground_z_ref
                end_coords = (neighbor_coords - coords) / 2 + coords
                end = Point()
                end.x = end_coords[0]
                end.y = end_coords[1]
                end.z = self.ground_z_ref
                edges.points.append(start)
                edges.points.append(end)

        self.edge_pub.publish(edges)

        # visualize nodes
        nodes = []
        for node_coords, utility in zip(self.robot.key_node_coords, self.robot.key_utility):
            nodes.append((node_coords[0], node_coords[1], self.ground_z_ref, utility))
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        nodes = point_cloud2.create_cloud(header, fields, nodes)
        self.node_pub.publish(nodes)

        # visualize frontiers
        frontiers = []
        for frontier in self.robot.frontier:
            frontiers.append((frontier[0], frontier[1], self.ground_z_ref))
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        frontiers = point_cloud2.create_cloud(header, fields, frontiers)
        self.frontier_pub.publish(frontiers)
        

if __name__ == '__main__':
    rospy.init_node('rl_planner', anonymous=True)
    rl_runner = Runner()
