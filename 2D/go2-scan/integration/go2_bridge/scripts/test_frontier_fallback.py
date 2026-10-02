#!/usr/bin/env python3
"""Offline regression tests for recoverable frontier deferral."""
import sys
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'algorithms/global_planning/ariadne/src/rl_planner/scripts'))

from exploration_continuity import FrontierContinuity  # noqa: E402


class Node:
    def __init__(self, neighbors=()):
        self.neighbor_set = set(neighbors)


class FrontierFallbackTest(unittest.TestCase):
    frontiers = {(5.0, 0.0), (5.0, 0.1)}

    @staticmethod
    def choose(continuity, now, nodes, frontiers=None, known_cells=100):
        return continuity.choose(
            None, frontiers or FrontierFallbackTest.frontiers, nodes,
            (0.0, 0.0), (0.0, 0.0), lambda _p: True,
            lambda _a, _b: True, set(), now, known_cells,
            observation_range=2.5, waypoint_range=5.0,
            min_distance=0.8, target_safe=lambda _p: True)

    def test_no_route_requires_retry_before_weak_completion(self):
        continuity = FrontierContinuity(defer_seconds=60, fallback_seconds=120)
        nodes = {(0.0, 0.0): Node()}
        self.assertIsNone(self.choose(continuity, 0, nodes))
        self.assertIsNone(self.choose(continuity, 1, nodes))
        self.assertEqual(len(continuity.current_deferred), 0)
        self.assertIsNone(self.choose(continuity, 2, nodes))
        self.assertFalse(continuity.weak_complete)
        self.assertIsNone(self.choose(continuity, 63, nodes))
        self.assertEqual(continuity.deferred_rechecks, 1)
        self.assertIsNone(self.choose(continuity, 123, nodes))
        self.assertTrue(continuity.weak_complete)
        normal, deferred, unresolved = continuity.frontier_classes(self.frontiers)
        self.assertEqual((len(normal), len(deferred), len(unresolved)), (0, 0, 2))

    def test_route_reappearance_recovers_immediately(self):
        continuity = FrontierContinuity(defer_seconds=60, fallback_seconds=120)
        for now in (0, 1, 2, 63, 123):
            self.choose(continuity, now, {(0.0, 0.0): Node()})
        nodes = {
            (0.0, 0.0): Node([(3.0, 0.0)]),
            (3.0, 0.0): Node([(0.0, 0.0)]),
        }
        self.assertIsNotNone(self.choose(continuity, 124, nodes))
        self.assertFalse(continuity.weak_complete)

    def test_meaningful_map_growth_reopens_grace_period(self):
        continuity = FrontierContinuity(defer_seconds=60, fallback_seconds=120)
        nodes = {(0.0, 0.0): Node()}
        self.choose(continuity, 0, nodes, known_cells=100)
        self.choose(continuity, 1, nodes, known_cells=100)
        self.choose(continuity, 2, nodes, known_cells=100)
        self.choose(continuity, 63, nodes, known_cells=100)
        self.choose(continuity, 123, nodes, known_cells=120)
        self.assertFalse(continuity.weak_complete)
        self.assertEqual(continuity.all_deferred_since, 123)

    def test_failed_retry_does_not_erase_unresolved_episode(self):
        continuity = FrontierContinuity(defer_seconds=60, fallback_seconds=120)
        nodes = {
            (0.0, 0.0): Node([(3.0, 0.0)]),
            (3.0, 0.0): Node([(0.0, 0.0)]),
        }
        self.assertIsNotNone(self.choose(continuity, 0, nodes))
        continuity.feedback((3.0, 0.0), False, 10, 'LOCAL_PLAN_TIMEOUT')
        self.assertIsNone(self.choose(continuity, 10, nodes))
        self.assertIsNone(self.choose(continuity, 70, nodes))
        self.assertIsNotNone(self.choose(continuity, 131, nodes))
        continuity.feedback((3.0, 0.0), False, 140, 'LOCAL_PLAN_TIMEOUT')
        self.assertIsNone(self.choose(continuity, 172, nodes))
        self.assertIsNone(self.choose(continuity, 190, nodes))
        self.assertTrue(continuity.weak_complete)

    def test_clearance_does_not_erase_an_existing_route(self):
        continuity = FrontierContinuity()
        nodes = {
            (0.0, 0.0): Node([(3.0, 0.0)]),
            (3.0, 0.0): Node([(0.0, 0.0)]),
        }
        result = continuity.choose(
            None, self.frontiers, nodes, (0.0, 0.0), (0.0, 0.0),
            lambda _p: True, lambda _a, _b: True, set(), 0, 100,
            observation_range=2.5, waypoint_range=5.0, min_distance=0.8,
            target_safe=lambda _p: False)
        self.assertIsNone(result)
        self.assertEqual(len(continuity.current_deferred), 0)
        self.assertFalse(continuity.weak_complete)

    def test_policy_can_advance_while_frontier_route_is_not_yet_built(self):
        continuity = FrontierContinuity()
        nodes = {
            (0.0, 0.0): Node([(2.0, 0.0)]),
            (2.0, 0.0): Node([(0.0, 0.0)]),
        }
        result = continuity.choose(
            (2.0, 0.0), {(10.0, 0.0)}, nodes,
            (0.0, 0.0), (0.0, 0.0), lambda _p: True,
            lambda _a, _b: True, set(), 0, 100,
            observation_range=2.5, waypoint_range=5.0, min_distance=0.8,
            target_safe=lambda _p: True)
        self.assertTrue((result == (2.0, 0.0)).all())

    def test_successful_advance_clears_stale_unroutable_evidence(self):
        continuity = FrontierContinuity()
        nodes = {
            (0.0, 0.0): Node([(2.0, 0.0)]),
            (2.0, 0.0): Node([(0.0, 0.0)]),
        }
        kwargs = dict(observation_range=2.5, waypoint_range=5.0,
                      min_distance=0.8, target_safe=lambda _p: True)
        self.assertIsNotNone(continuity.choose(
            (2.0, 0.0), {(10.0, 0.0)}, nodes, (0.0, 0.0),
            (0.0, 0.0), lambda _p: True, lambda _a, _b: True,
            set(), 0, 100, **kwargs))
        continuity.feedback((2.0, 0.0), True, 4.0, 'REACHED')
        self.assertIsNotNone(continuity.choose(
            (2.0, 0.0), {(10.0, 0.0)}, nodes, (0.0, 0.0),
            (0.0, 0.0), lambda _p: True, lambda _a, _b: True,
            set(), 4.0, 110, **kwargs))
        self.assertEqual(len(continuity.current_deferred), 0)

    def test_moving_frontier_keeps_committed_branch_over_rl_backtrack(self):
        continuity = FrontierContinuity()
        continuity.active = np.array([(5.0, 0.0), (5.0, 0.1)])
        continuity.heading = np.array([1.0, 0.0])
        continuity.last_progress = 0.0
        nodes = {
            (4.0, 0.0): Node([(6.0, 0.0), (2.0, 0.0)]),
            (6.0, 0.0): Node([(4.0, 0.0)]),
            (2.0, 0.0): Node([(4.0, 0.0), (0.0, 0.0)]),
            (0.0, 0.0): Node([(2.0, 0.0)]),
        }
        result = continuity.choose(
            (0.0, 0.0), {(7.0, 1.0), (7.0, 1.1), (0.0, 0.0)}, nodes,
            (4.0, 0.0), (4.0, 0.0), lambda _p: True,
            lambda _a, _b: True, set(), 1, 120,
            observation_range=2.5, waypoint_range=5.0, min_distance=0.8,
            target_safe=lambda _p: True)
        self.assertGreater(result[0], 4.0)
        self.assertEqual(continuity.reason, 'continue_committed_branch:graph_route')

    def test_backtrack_waits_until_committed_branch_is_confirmed_gone(self):
        continuity = FrontierContinuity()
        continuity.active = np.array([(5.0, 0.0), (5.0, 0.1)])
        continuity.heading = np.array([1.0, 0.0])
        nodes = {
            (4.0, 0.0): Node([(2.0, 0.0)]),
            (2.0, 0.0): Node([(4.0, 0.0), (0.0, 0.0)]),
            (0.0, 0.0): Node([(2.0, 0.0)]),
        }
        args = ((0.0, 0.0), {(-1.0, 0.0), (-1.0, 0.1)}, nodes,
                (4.0, 0.0), (4.0, 0.0), lambda _p: True,
                lambda _a, _b: True, set())
        self.assertIsNone(continuity.choose(*args, 0.0, 100,
                          observation_range=2.5, waypoint_range=5.0,
                          min_distance=0.8, target_safe=lambda _p: True))
        self.assertIsNone(continuity.choose(*args, 1.0, 100,
                          observation_range=2.5, waypoint_range=5.0,
                          min_distance=0.8, target_safe=lambda _p: True))
        result = continuity.choose(*args, 2.1, 100,
                                   observation_range=2.5, waypoint_range=5.0,
                                   min_distance=0.8, target_safe=lambda _p: True)
        self.assertLess(result[0], 4.0)


if __name__ == '__main__':
    unittest.main()
