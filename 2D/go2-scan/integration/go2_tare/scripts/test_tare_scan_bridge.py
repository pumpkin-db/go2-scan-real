#!/usr/bin/env python3
"""Offline tests for simulation-style latest-intent SCAN handoff."""
import importlib.util
from pathlib import Path
from unittest import mock

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from scan_planner.msg import GoalFeedback


class Published:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def load_bridge():
    spec = importlib.util.spec_from_file_location('tare_scan_bridge', Path(__file__).with_name('tare_scan_bridge.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def waypoint(stamp, x):
    msg = PointStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = rospy.Time(stamp)
    msg.point.x = x
    return msg


def feedback(stamp, state):
    msg = GoalFeedback()
    msg.request_header.stamp = stamp
    msg.state = state
    return msg


def make_goals(bridge):
    goals = bridge.Goals.__new__(bridge.Goals)
    goals.pose = Odometry()
    goals.pose.header.frame_id = 'map'
    goals.pose.pose.pose.position.x = 0.0
    goals.pending = None
    goals.last_id = 0
    goals.active_id = None
    goals.lock = bridge.threading.RLock()
    goals.min_goal_distance = 0.6
    goals.use_path = False
    goals.goal_pub = Published()
    goals.path_pub = Published()
    goals.display_pub = Published()
    return goals


def route(points):
    msg = RosPath()
    msg.header.frame_id = 'map'
    for x, y in points:
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)
    return msg


def test_path_preserves_corner_waits_and_uses_latest_after_failure():
    bridge = load_bridge()
    goals = make_goals(bridge)
    goals.use_path = True
    with mock.patch.object(rospy, 'get_param', return_value=True), \
         mock.patch.object(rospy.Time, 'now', side_effect=[rospy.Time(300), rospy.Time(301)]):
        goals.goal(route([(0, 0), (2, 0), (2, 2)]))
        sent = goals.path_pub.messages[-1]
        assert [(p.pose.position.x, p.pose.position.y) for p in sent.poses] == [(0, 0), (2, 0), (2, 2)]
        assert sent.header.frame_id == 'world'
        assert all(p.header == sent.header for p in sent.poses)
        assert not goals.goal_pub.messages
        stamp = sent.header.stamp
        goals.goal(route([(0, 0), (2, 0), (2, 3)]))
        goals.goal(route([(0, 0), (2, 0), (2, 4)]))
        goals.feedback(feedback(stamp, GoalFeedback.ACTIVE))
        goals.feedback(feedback(rospy.Time(299), GoalFeedback.FAILED))
        assert len(goals.path_pub.messages) == 1
        goals.pose.pose.pose.position.x = 2
        goals.pose.pose.pose.position.y = 1
        goals.feedback(feedback(stamp, GoalFeedback.FAILED))
        assert len(goals.path_pub.messages) == 2
        sent = goals.path_pub.messages[-1]
        assert [(p.pose.position.x, p.pose.position.y) for p in sent.poses] == [(2, 1), (2, 4)]
        assert sent.header.stamp > stamp
        assert goals.display_pub.messages[-1].point == sent.poses[-1].pose.position


def test_path_validation_and_reanchor():
    bridge = load_bridge()
    goals = make_goals(bridge)
    goals.use_path = True
    with mock.patch.object(rospy, 'get_param', return_value=True), mock.patch.object(rospy, 'logwarn_throttle'):
        for msg in [route([]), route([(0, 0)]), route([(0, 0), (.6, 0)]),
                    route([(0, 0), (float('nan'), 1), (2, 2)])]:
            goals.goal(msg)
        assert not goals.path_pub.messages
        assert goals.active_id is None
    pos = goals.pose.pose.pose.position
    pos.x, pos.y = 1, .1
    result = bridge.Goals.remaining_path(route([(0, 0), (2, 0), (2, 3)]), pos)
    assert [(p.pose.position.x, p.pose.position.y) for p in result.poses] == [(1, .1), (1, 0), (2, 0), (2, 3)]


def test_latest_goal_waits_for_matching_terminal_result():
    bridge = load_bridge()
    goals = make_goals(bridge)

    with mock.patch.object(rospy, 'get_param', return_value=True), \
         mock.patch.object(rospy.Time, 'now', side_effect=[rospy.Time(100), rospy.Time(101)]):
        goals.goal(waypoint(10, 2.0))
        first_id = goals.active_id
        assert len(goals.goal_pub.messages) == 1

        goals.goal(waypoint(11, 3.0))
        goals.goal(waypoint(12, 4.0))
        assert len(goals.goal_pub.messages) == 1
        assert goals.pending.point.x == 4.0

        goals.feedback(feedback(rospy.Time(99), GoalFeedback.FAILED))
        goals.feedback(feedback(rospy.Time(first_id // 1000000000, first_id % 1000000000), GoalFeedback.ACTIVE))
        assert len(goals.goal_pub.messages) == 1

        stamp = rospy.Time(first_id // 1000000000, first_id % 1000000000)
        goals.feedback(feedback(stamp, GoalFeedback.SUCCEEDED))
        assert len(goals.goal_pub.messages) == 2
        assert goals.goal_pub.messages[-1].pose.position.x == 4.0
        assert goals.active_id is not None


def test_noop_goal_is_not_sent_to_scan():
    bridge = load_bridge()
    goals = make_goals(bridge)
    with mock.patch.object(rospy, 'get_param', return_value=True), \
         mock.patch.object(rospy, 'logwarn_throttle'):
        goals.goal(waypoint(10, 0.2))
    assert not goals.goal_pub.messages
    assert goals.active_id is None


def test_failed_goal_releases_latest_intent():
    bridge = load_bridge()
    goals = make_goals(bridge)
    with mock.patch.object(rospy, 'get_param', return_value=True), \
         mock.patch.object(rospy.Time, 'now', side_effect=[rospy.Time(200), rospy.Time(201)]):
        goals.goal(waypoint(20, 2.0))
        first_id = goals.active_id
        goals.goal(waypoint(21, 5.0))
        stamp = rospy.Time(first_id // 1000000000, first_id % 1000000000)
        goals.feedback(feedback(stamp, GoalFeedback.FAILED))
    assert len(goals.goal_pub.messages) == 2
    assert goals.goal_pub.messages[-1].pose.position.x == 5.0


if __name__ == '__main__':
    test_latest_goal_waits_for_matching_terminal_result()
    test_noop_goal_is_not_sent_to_scan()
    test_failed_goal_releases_latest_intent()
    test_path_preserves_corner_waits_and_uses_latest_after_failure()
    test_path_validation_and_reanchor()
    print('TARE -> SCAN latest-intent handoff: OK')
