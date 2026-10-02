"""Focused offline regressions for the actual-trail return chain (no ROS master)."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'algorithms/global_planning/ariadne/src/rl_planner/scripts'))
from exploration_continuity import FrontierContinuity
from ariadne_goal_bridge import FeedbackWaypointBridge
from nav_msgs.msg import OccupancyGrid
import test_goal_feedback as feedback_fixtures
from geometry_msgs.msg import PointStamped
from scan_planner.msg import GoalFeedback
import rospy


class HistoryReturnTests(unittest.TestCase):
    def setUp(self):
        self.c = FrontierContinuity()
        self.c.trail = [np.array([x, 0.]) for x in range(5)]

    def choose(self, robot=(4., 0.), frontiers=None, proposed=None,
               excluded=(), free=lambda p: True, clear=lambda a, b: False):
        return self.c.choose(proposed, frontiers or {(-4., 0.)}, {}, robot,
                             robot, free, clear, excluded, 0., 100)

    def test_return_before_frontier_is_visible(self):
        np.testing.assert_allclose(self.choose(), (0., 0.))
        self.assertTrue(self.c.route_uses_trail)

    def test_no_shuttle_when_closest_history_point_reached(self):
        self.assertIsNone(self.choose(robot=(0., 0.)))

    def test_do_not_cross_history_break(self):
        self.c.trail_breaks = {4}
        self.assertIsNone(self.choose())

    def test_no_trail_does_not_invent_a_route(self):
        self.c.trail = []
        self.assertIsNone(self.choose())

    def test_policy_choice_is_not_overridden_by_new_fallback(self):
        np.testing.assert_allclose(self.choose(proposed=(5., 1.)), (5., 1.))

    def test_occupied_or_excluded_endpoints_not_selected(self):
        result = self.choose(excluded={(0., 0.)}, free=lambda p: p[0] != 1.)
        np.testing.assert_allclose(result, (2., 0.))

    def test_stops_at_corner(self):
        self.c.trail = [np.array(p, dtype=float) for p in
                        [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)]]
        np.testing.assert_allclose(self.choose(robot=(2., 2.)), (2., 0.))

    def test_visible_frontier_route_still_works(self):
        np.testing.assert_allclose(self.choose(frontiers={(-1., 0.)},
                                              clear=lambda a, b: True), (0., 0.))


class BridgeReturnTests(unittest.TestCase):
    def setUp(self):
        self.b = FeedbackWaypointBridge.__new__(FeedbackWaypointBridge)
        grid = OccupancyGrid()
        grid.info.resolution = .1
        grid.info.width, grid.info.height = 81, 41
        grid.info.origin.position.x = grid.info.origin.position.y = -2.
        data = np.zeros((41, 81), dtype=int)
        data[:, 40] = 100  # old obstacle divides x=0 from robot at x=4
        grid.data = data.ravel().tolist()
        self.b.projected_map = grid
        self.b.robot_xy = (4., 0.)
        self.b.decision = SimpleNamespace(point=SimpleNamespace(x=0., y=0.))
        self.b.failed = []
        self.b.target_search_radius = 5.
        self.b.target_min_robot_distance = .8
        self.b.target_min_clearance = .4
        self.b.local_obstacle_clearance = lambda x, y: (False, None)

    def test_safe_requested_goal_not_relocated_by_stale_component(self):
        self.assertTrue(self.b.target_is_safe(self.b.projected_map, 0., 0.))
        self.assertEqual(self.b.choose(), (0., 0.))

    def test_failed_requested_goal_is_not_retried(self):
        self.b.failed = [(0., 0.)]
        self.assertNotEqual(self.b.choose(), (0., 0.))

    def test_unsafe_goal_still_requires_clearance(self):
        self.b.decision.point.x = 2.05
        result = self.b.choose()
        self.assertIsNotNone(result)
        self.assertTrue(self.b.target_is_safe(self.b.projected_map, *result))
        self.assertNotEqual(result, (2.05, 0.))


class ReturnFeedbackTests(unittest.TestCase):
    # Reuse only ROS mocks/setup, not obsolete assertions about old timeouts.
    setUp = feedback_fixtures.FeedbackTests.setUp
    advance = feedback_fixtures.FeedbackTests.advance

    def test_pending_goal_not_replaced_and_success_keeps_ar_id(self):
        self.b.motion = False
        request = PointStamped()
        request.header.stamp = rospy.Time(90)
        request.point.x = 3.
        self.b.waypoint_cb(request)
        self.advance(.1)
        self.assertEqual(self.b.decision.header.stamp, rospy.Time(80))
        self.assertEqual(len(self.b.path_pub.messages), 1)
        feedback = GoalFeedback()
        feedback.state, feedback.reason = feedback.SUCCEEDED, 'REACHED'
        feedback.request_header.stamp = rospy.Time(1)
        self.b.events.append(('feedback', feedback))
        self.advance(.1)
        self.assertIsNotNone(self.b.decision)
        feedback.request_header = self.b.attempt.header
        self.b.events.append(('feedback', feedback))
        self.advance(.1)
        self.assertIsNone(self.b.decision)
        self.assertEqual(self.b.result.request_header.stamp, rospy.Time(80))
        self.assertEqual(self.b.result.state, feedback.SUCCEEDED)
        self.b.waypoint_cb(request)
        self.advance(.1)
        self.assertEqual(self.b.decision.header.stamp, rospy.Time(90))

    def test_relocated_retry_success_releases_original_ar_request(self):
        self.b.motion = False
        original = self.b.current
        feedback = GoalFeedback()
        feedback.state, feedback.reason = feedback.FAILED, 'LOCAL_PLAN_TIMEOUT'
        feedback.request_header = self.b.attempt.header
        self.b.events.append(('feedback', feedback))
        self.advance(.1)
        self.assertNotEqual(self.b.current, original)
        self.assertEqual(self.b.counts['retry_sent'], 1)
        feedback = GoalFeedback()
        feedback.state, feedback.reason = feedback.SUCCEEDED, 'REACHED'
        feedback.request_header = self.b.attempt.header
        self.b.events.append(('feedback', feedback))
        self.advance(.1)
        self.assertIsNone(self.b.decision)
        self.assertEqual(self.b.result.request_header.stamp, rospy.Time(80))


if __name__ == '__main__':
    unittest.main()
