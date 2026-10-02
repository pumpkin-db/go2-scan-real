"""Execute the actual Runner decision method without loading torch/ROS nodes."""
import ast
import json
import sys
import time
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / 'algorithms/global_planning/ariadne/src/rl_planner/scripts'
sys.path.insert(0, str(SCRIPTS))
import parameter
from utils import get_cell_position_from_coords, check_collision
from exploration_continuity import FrontierContinuity
from frontier_grid_fallback import grid_frontier_waypoint
import test_frontier_grid as grid_fixtures
from ariadne_goal_bridge import FeedbackWaypointBridge
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from scan_planner.msg import GoalFeedback

tree = ast.parse((SCRIPTS / 'rl_planner.py').read_text())
method = next(m for c in tree.body if isinstance(c, ast.ClassDef) and c.name == 'Runner'
              for m in c.body if isinstance(m, ast.FunctionDef) and m.name == 'continuous_frontier_waypoint')
rospy = SimpleNamespace(get_param=lambda key, default: default, loginfo=lambda *a: None)
namespace = dict(np=np, time=time, json=json, parameter=parameter, rospy=rospy,
                 String=lambda s:s, get_cell_position_from_coords=get_cell_position_from_coords,
                 check_collision=check_collision, grid_frontier_waypoint=grid_frontier_waypoint)
exec(compile(ast.Module(body=[method], type_ignores=[]), str(SCRIPTS/'rl_planner.py'), 'exec'), namespace)
decision = namespace['continuous_frontier_waypoint']
run_method = next(m for c in tree.body if isinstance(c, ast.ClassDef) and c.name == 'Runner'
                  for m in c.body if isinstance(m, ast.FunctionDef) and m.name == 'run')
exec(compile(ast.Module(body=[run_method], type_ignores=[]), str(SCRIPTS/'rl_planner.py'), 'exec'), namespace)


class GridIntegrationTest(unittest.TestCase):
    def test_reached_policy_point_enters_grid_recovery(self):
        result = decision(self.r, self.r.robot_location.copy(), self.frontiers)
        self.assertIsNotNone(result)
        self.assertGreater(np.linalg.norm(result-self.r.robot_location), .8)
        self.assertIn('grid_frontier', self.r.continuity.reason)

    def test_reached_point_with_closed_wall_is_not_republished(self):
        self.grid[:,60]=100
        self.assertIsNone(decision(self.r,self.r.robot_location.copy(),self.frontiers))

    def setUp(self):
        self.param_patch = patch.object(parameter, 'THR_NEXT_WAYPOINT', 4.)
        self.param_patch.start(); self.addCleanup(self.param_patch.stop)
        fixture = grid_fixtures.GridFallbackTest(); fixture.setUp()
        self.grid, self.frontiers = fixture.grid, {(10.05,3.05),(10.05,3.15)}
        self.r = SimpleNamespace(goal_feedback_enabled=True, continuity=FrontierContinuity(),
            robot_location=fixture.robot, policy_blocked_nodes=set(),
            map_info=SimpleNamespace(map=self.grid, map_origin_x=0., map_origin_y=0., cell_size=.1),
            robot=SimpleNamespace(node_manager=SimpleNamespace(nodes_dict=[])),
            diagnostic_pub=SimpleNamespace(publish=lambda x:None))

    def test_real_decision_reaches_grid_fallback_only_when_empty(self):
        result = decision(self.r, None, self.frontiers)
        self.assertIsNotNone(result)
        self.assertIn('grid_frontier', self.r.continuity.reason)
        b = FeedbackWaypointBridge.__new__(FeedbackWaypointBridge)
        b.projected_map=OccupancyGrid(); b.projected_map.info.resolution=.1
        b.projected_map.info.height,b.projected_map.info.width=self.grid.shape
        b.projected_map.data=self.grid.ravel().tolist()
        b.robot_xy=tuple(self.r.robot_location); b.failed=[]
        b.decision=SimpleNamespace(point=SimpleNamespace(x=result[0],y=result[1]))
        b.target_min_clearance=.4; b.target_min_robot_distance=.8; b.target_search_radius=5.
        b.local_obstacle_clearance=lambda x,y:(False,None)
        self.assertEqual(b.choose(),tuple(result))

    def test_policy_and_history_results_skip_grid_search(self):
        for proposed in (None, np.array([4.,3.])):
            self.r.continuity.choose=Mock(return_value=np.array([4.,3.]))
            with patch.dict(namespace, grid_frontier_waypoint=Mock(side_effect=AssertionError('unexpected grid call'))):
                np.testing.assert_allclose(decision(self.r,proposed,self.frontiers),(4.,3.))

    def test_actual_history_chain_precedes_grid(self):
        self.r.continuity.trail=[np.array([x,3.05]) for x in np.arange(3.05,8.1,.5)]
        with patch.dict(namespace, grid_frontier_waypoint=Mock(side_effect=AssertionError('unexpected grid call'))):
            self.assertIsNotNone(decision(self.r,None,self.frontiers))
        self.assertTrue(self.r.continuity.route_uses_trail)

    def test_closed_wall_stays_no_goal(self):
        self.grid[:,60]=100
        self.assertIsNone(decision(self.r,None,self.frontiers))

    def test_bridge_parameters_are_reused(self):
        mock=Mock(return_value=(None,[]))
        with patch.dict(namespace, grid_frontier_waypoint=mock), patch.object(
                rospy,'get_param',side_effect=lambda k,d: .6 if k.endswith('clearance') else 1.1):
            decision(self.r,None,self.frontiers)
        self.assertEqual(mock.call_args.kwargs['clearance'], .6)
        self.assertEqual(mock.call_args.kwargs['min_distance'], 1.1)

    def test_actual_runner_waits_and_matching_success_unlocks(self):
        r=self.r
        r.odom_samples=deque(); r.goal_results=deque()
        r.pending_goal=PointStamped(); r.pending_goal.header.stamp.secs=123
        r.directional=None; r.reset_requested=False; r.paused=False; r.start=None
        r.failed_until={}; r.waypoint_pub=Mock(); r.publish_status=Mock()
        with patch.dict(namespace, grid_frontier_waypoint=Mock(side_effect=AssertionError('pending goal replanned'))):
            namespace['run'](r)
        self.assertIsNotNone(r.pending_goal)
        self.assertEqual(r.waypoint_pub.publish.call_count,1)
        event=GoalFeedback(); event.state=event.SUCCEEDED; event.request_header=r.pending_goal.header
        r.goal_results.append(event)
        namespace['run'](r)
        self.assertIsNone(r.pending_goal)


if __name__ == '__main__':
    unittest.main()
