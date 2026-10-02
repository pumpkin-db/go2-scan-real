"""Independent topology oracle and deterministic selection/ideal-motion tests."""
import sys
import unittest
from collections import deque
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'algorithms/global_planning/ariadne/src/rl_planner/scripts'))
from frontier_grid_fallback import grid_frontier_waypoint


def reachable(grid, start):
    result, todo = {start}, deque([start])
    while todo:
        y, x = todo.popleft()
        for p in ((y-1,x), (y+1,x), (y,x-1), (y,x+1)):
            if 0 <= p[0] < grid.shape[0] and 0 <= p[1] < grid.shape[1] \
                    and grid[p] == 0 and p not in result:
                result.add(p); todo.append(p)
    return result


class GridFallbackTest(unittest.TestCase):
    def setUp(self):
        self.grid = np.zeros((100, 140), dtype=int)
        self.grid[[0,-1], :] = 100
        self.grid[:, [0,-1]] = 100
        self.robot = np.array([3.05, 3.05])
        self.regions = [np.array([[10.05,3.05], [10.05,3.15]])]

    def plan(self, **kw):
        return grid_frontier_waypoint(self.grid, (0,0), .1, self.robot,
                                      self.regions, max_leg=4., **kw)

    def check_route(self, target, route, origin=(0,0), res=.1):
        self.assertIsNotNone(target)
        cells = np.floor((np.asarray(route)-origin)/res).astype(int)[:, ::-1]
        self.assertTrue((self.grid[cells[:,0], cells[:,1]] == 0).all())
        self.assertTrue((np.abs(np.diff(cells, axis=0)).sum(axis=1) == 1).all())
        self.assertIn(tuple(cells[-1]), reachable(self.grid, tuple(cells[0])))
        self.assertGreater(np.linalg.norm(target-self.robot), .8)
        self.assertLessEqual(np.linalg.norm(target-self.robot), 4.1)
        # Independent exact point-to-occupied-square clearance, not EDT.
        oy, ox = np.nonzero(self.grid >= 50)
        centers = np.asarray(origin)+(np.column_stack((ox,oy))+.5)*res
        boundary_distance = np.linalg.norm(np.maximum(abs(centers-target)-res/2,0), axis=1)
        self.assertTrue((boundary_distance > .4).all())

    def test_closed_wall_returns_no_target(self):
        self.grid[:,60] = 100
        self.assertIsNone(self.plan()[0])

    def test_door_route_goes_around_wall(self):
        self.grid[:,60] = 100
        self.grid[65:80,60] = 0
        target, route = self.plan()
        self.check_route(target, route)
        self.assertGreater(max(np.asarray(route)[:,1]), 6.4)
        self.assertGreater(target[1], self.robot[1])

    def test_near_unreachable_region_does_not_hide_far_reachable(self):
        self.grid[:,60] = 100
        self.regions = [np.array([[6.65,3.05]]), np.array([[3.05,8.55]])]
        target, route = self.plan()
        self.check_route(target, route)
        self.assertLess(np.linalg.norm(route[-1]-self.regions[1][0]), .81)

    def test_unknown_wall_is_not_traversed(self):
        self.grid[:,60] = -1
        self.assertIsNone(self.plan()[0])

    def test_occupied_unknown_and_outside_robot_have_no_seed(self):
        for value in (100,-1):
            self.grid[30,30] = value
            self.assertIsNone(self.plan()[0])
        self.robot = np.array([-1.,3.])
        self.assertIsNone(self.plan()[0])

    def test_diagonal_touch_is_not_a_passage(self):
        self.grid[:] = 100
        self.grid[30,30] = self.grid[31,31] = 0
        self.regions = [np.array([[3.15,3.15]])]
        self.assertIsNone(self.plan(clearance=0., min_distance=.01)[0])

    def test_too_narrow_has_no_clearance_target(self):
        self.grid[:] = 100
        self.grid[28:34,10:130] = 0
        self.assertIsNone(self.plan()[0])

    def test_excluded_waypoint_not_reissued(self):
        target, _ = self.plan()
        other, route = self.plan(excluded=[target])
        self.check_route(other, route)
        self.assertGreaterEqual(np.linalg.norm(other-target), .6)

    def test_no_frontier_no_goal(self):
        self.regions = []
        self.assertIsNone(self.plan()[0])

    def test_repeated_ideal_steps_reach_door_frontier_without_loop(self):
        self.grid[:,60] = 100
        self.grid[65:80,60] = 0
        visited = set()
        for _ in range(20):
            if np.min(np.linalg.norm(self.regions[0]-self.robot, axis=1)) <= 1.7:
                return
            target, route = self.plan()
            self.check_route(target, route)
            key = tuple(np.round(target,3))
            self.assertNotIn(key, visited)
            visited.add(key)
            self.robot = target  # ideal execution only, not SCAN simulation
        self.fail('failed to reach frontier approach in 20 ideal steps')

    def test_100_seeded_maps_against_independent_bfs(self):
        produced = 0
        for seed in range(100):
            rng = np.random.default_rng(seed)
            self.grid = np.where(rng.random((40,40)) < .12, 100, 0)
            self.grid[:8,:8] = 0
            self.robot = np.array([.65,.65])
            free = np.argwhere(self.grid == 0)
            sampled = free[rng.choice(len(free), 12, replace=False)]
            self.regions = [(p[::-1][None,:]+.5)*.1 for p in sampled]
            target, route = self.plan()
            if target is not None:
                produced += 1
                self.check_route(target, route)
        self.assertGreater(produced, 0)  # prevent vacuous all-None success

    def test_door_approach_with_point_eight_arrival_tolerance(self):
        self.grid[:,60] = 100
        self.grid[65:80,60] = 0
        for _ in range(60):
            if np.min(np.linalg.norm(self.regions[0]-self.robot, axis=1)) <= 1.7:
                return
            target, route = self.plan()
            self.check_route(target, route)
            delta = target-self.robot
            length = np.linalg.norm(delta)
            self.robot = self.robot + delta/length*(length-.79)
        self.fail('0.8m arrival tolerance prevents progress')

    def test_translated_origin_preserves_world_coordinates(self):
        target, _ = self.plan()
        shift=np.array([-17.3, 8.7])
        other, _ = grid_frontier_waypoint(self.grid, shift, .1,
            self.robot+shift, [r+shift for r in self.regions], max_leg=4.)
        np.testing.assert_allclose(other-shift,target,atol=1e-8)

    def test_door_closed_then_open_replans_without_cooldown(self):
        self.grid[:,60] = 100
        self.assertIsNone(self.plan()[0])
        self.grid[65:80,60] = 0
        self.check_route(*self.plan())
        self.grid[:,60] = 100
        self.assertIsNone(self.plan()[0])


if __name__ == '__main__':
    unittest.main()
