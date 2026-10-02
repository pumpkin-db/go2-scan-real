#!/usr/bin/env python3
"""Offline geometry/feedback regressions for the optional direction strategy."""
import sys
import unittest
import ast
import time
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'algorithms/global_planning/ariadne/src/rl_planner/scripts'))
from directional_exploration import DirectionalExploration
from exploration_continuity import FrontierContinuity


def graph(*branches):
    nodes = {}
    for branch in branches:
        for point in branch:
            nodes.setdefault(point, SimpleNamespace(neighbor_set=set()))
        for a, b in zip(branch, branch[1:]):
            nodes[a].neighbor_set.add(b)
            nodes[b].neighbor_set.add(a)
    return nodes


class DirectionTest(unittest.TestCase):
    def setUp(self):
        self.strategy = DirectionalExploration()
        self.routes = FrontierContinuity()
        self.strategy.dispatched((-2., 0.))
        self.strategy.feedback(True, (0., 0.), [])

    def choose(self, nodes, frontiers, excluded=()):
        return self.strategy.choose(frontiers, nodes, (0., 0.), (0., 0.),
            lambda p: True, lambda a, b: True, set(excluded), self.routes,
            observation_range=2.5)

    def test_more_side_frontiers_do_not_override_forward(self):
        nodes = graph([(0., 0.), (2., 0.), (4., 0.)],
                      [(0., 0.), (0., 2.), (0., 4.)])
        frontiers = {(6., 0.)} | {(0.01*i, 5.) for i in range(50)}
        np.testing.assert_allclose(self.choose(nodes, frontiers), (4., 0.))

    def test_right_angle_bend_is_available(self):
        nodes = graph([(0., 0.), (0., 2.), (0., 4.)])
        np.testing.assert_allclose(self.choose(nodes, {(0., 6.)}), (0., 4.))
        self.strategy.dispatched((0., 0.))
        self.strategy.feedback(True, (0., 4.), [(0., 2.), (0., 3.), (0., 4.)])
        np.testing.assert_allclose(self.strategy.heading, (0., 1.))

    def test_failure_keeps_direction_and_tries_other_point(self):
        nodes = graph([(0., 0.), (2., 0.), (4., 0.)])
        self.strategy.dispatched((0., 0.))
        self.strategy.feedback(False, (0., 0.), [])
        np.testing.assert_allclose(self.choose(nodes, {(4., 0.)}, [(4., 0.)]), (2., 0.))
        np.testing.assert_allclose(self.strategy.heading, (1., 0.))

    def test_exhausted_candidates_release_to_rl_immediately(self):
        nodes = graph([(0., 0.), (2., 0.), (4., 0.)])
        self.assertIsNone(self.choose(nodes, {(4., 0.)}, [(2., 0.), (4., 0.)]))
        self.assertIsNone(self.strategy.heading)

    def test_only_backtracking_releases_to_existing_return_logic(self):
        nodes = graph([(0., 0.), (-2., 0.), (-4., 0.)])
        self.assertIsNone(self.choose(nodes, {(-6., 0.)}))
        self.assertIsNone(self.strategy.heading)

    def test_route_bearing_handles_u_shaped_corridor(self):
        nodes = graph([(0., 0.), (2., 0.), (4., 0.), (4., 3.),
                       (4., 6.), (1., 6.), (-2., 6.)])
        np.testing.assert_allclose(self.choose(nodes, {(-3., 6.)}), (4., 0.))

    def test_initial_selection_and_tolerance_only_success_stay_with_rl(self):
        strategy = DirectionalExploration()
        strategy.dispatched((0., 0.))
        strategy.feedback(True, (0.1, 0.), [])
        self.assertIsNone(strategy.heading)

    def test_disconnected_map_releases_without_waiting(self):
        nodes = graph([(0., 0.)], [(4., 0.), (6., 0.)])
        self.assertIsNone(self.choose(nodes, {(7., 0.)}))

    def test_no_frontiers_does_not_mark_exploration_complete(self):
        self.assertIsNone(self.choose(graph([(0., 0.)]), set()))
        self.assertIsNone(self.strategy.heading)


class DecisionFlowTest(unittest.TestCase):
    """Execute the real Runner.run with ROS I/O mocked, without starting ROS."""
    def setUp(self):
        source = ROOT / 'algorithms/global_planning/ariadne/src/rl_planner/scripts/rl_planner.py'
        runner = next(n for n in ast.parse(source.read_text()).body
                      if isinstance(n, ast.ClassDef) and n.name == 'Runner')
        method = next(n for n in runner.body if isinstance(n, ast.FunctionDef) and n.name == 'run')
        namespace = dict(np=np, time=time, rospy=Mock(get_param=Mock(return_value=True)),
                         parameter=SimpleNamespace(AVOID_OSCILLATION=False, UTILITY_RANGE=2.5),
                         get_frontier_in_map=lambda _: {(4., 0.)})
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
        self.run = namespace['run']
        self.r = SimpleNamespace(
            odom_samples=deque(), goal_results=deque(), goal_feedback_enabled=True,
            pending_goal=None, reset_requested=False, paused=False, start=np.array([0., 0.]),
            done=False, failed_until={}, escape_mode=False, save_mode=False, escape_arrived=False,
            next_waypoint=None, next_waypoint_list=[], history_waypoint_list=[], step=0,
            publish_graph=False, robot_location=np.array([0., 0.]),
            map_info=SimpleNamespace(map=np.zeros((10, 10))),
            continuity=Mock(), directional=Mock(heading=np.array([1., 0.])),
            update_planning_graph=Mock(return_value=np.array([0., 0.])),
            publish_waypoint=Mock(), waypoint_wrapper=lambda p: p,
            waypoint_pub=Mock(),
            continuous_frontier_waypoint=Mock(return_value=np.array([3., 0.])),
            robot=SimpleNamespace(location=np.array([0., 0.]), key_utility=[1],
                node_manager=SimpleNamespace(nodes_dict=[]),
                get_observation=Mock(return_value=None),
                select_next_waypoint=Mock(return_value=(np.array([2., 0.]), 0))))

    def test_pending_scan_goal_prevents_both_selectors(self):
        self.r.pending_goal = object()
        self.run(self.r)
        self.r.directional.choose.assert_not_called()
        self.r.robot.select_next_waypoint.assert_not_called()
        self.r.publish_waypoint.assert_not_called()

    def test_direction_selection_skips_rl(self):
        self.r.directional.choose.return_value = np.array([4., 0.])
        self.run(self.r)
        np.testing.assert_allclose(self.r.publish_waypoint.call_args[0][0], (4., 0.))
        self.r.robot.select_next_waypoint.assert_not_called()

    def test_empty_direction_falls_back_in_same_tick(self):
        self.r.directional.choose.return_value = None
        self.run(self.r)
        self.r.robot.select_next_waypoint.assert_called_once()
        np.testing.assert_allclose(self.r.publish_waypoint.call_args[0][0], (2., 0.))

    def test_disabled_mode_preserves_original_selector(self):
        self.r.directional = None
        self.run(self.r)
        self.r.robot.select_next_waypoint.assert_called_once()
        self.r.continuous_frontier_waypoint.assert_called_once()
        np.testing.assert_allclose(self.r.publish_waypoint.call_args[0][0], (3., 0.))

    def test_missing_free_anchor_uses_recovery_without_stale_policy(self):
        self.r.update_planning_graph.return_value = None
        self.run(self.r)
        self.r.robot.select_next_waypoint.assert_not_called()
        self.r.directional.choose.assert_not_called()
        self.r.continuous_frontier_waypoint.assert_called_once()
        np.testing.assert_allclose(self.r.publish_waypoint.call_args[0][0], (3., 0.))

    def test_missing_anchor_and_no_recovery_does_not_publish_old_goal(self):
        self.r._log_stop = Mock()
        self.r.update_planning_graph.return_value = None
        self.r.continuous_frontier_waypoint.return_value = None
        self.run(self.r)
        self.r.robot.select_next_waypoint.assert_not_called()
        self.r.publish_waypoint.assert_not_called()


if __name__ == '__main__':
    unittest.main()
