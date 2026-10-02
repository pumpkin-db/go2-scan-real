"""Synthetic selection comparison only; no ROS nodes, RL inference or SCAN run.

Graph input deliberately represents the observed zero-utility/disconnected state.
The simple prototype below is NOT imported by production launchers.
"""
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy.ndimage import label
from scipy.spatial import cKDTree
from nav_msgs.msg import OccupancyGrid
from ariadne_goal_bridge import FeedbackWaypointBridge

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'algorithms/global_planning/ariadne/src/rl_planner/scripts'))
from exploration_continuity import FrontierContinuity


def prototype(c, frontiers, b, grid):
    """Nearer metre-band first, then larger cluster; endpoint checks only."""
    ys, xs = np.nonzero(grid == 0)
    points = np.column_stack(((xs + .5) * .1, (ys + .5) * .1))
    regions = c.clusters(frontiers)
    regions.sort(key=lambda r: (int(np.min(np.linalg.norm(r-b.robot_xy, axis=1))), -len(r)))
    for region in regions:
        near = cKDTree(region).query(points)[0]
        indices = np.flatnonzero(near <= .8)
        indices = sorted(indices, key=lambda i: (near[i], np.linalg.norm(points[i]-b.robot_xy)))
        for i in indices:
            p = tuple(points[i])
            if b.target_is_far_enough(*p) and b.target_is_safe(b.projected_map, *p):
                return p
    return None


def run(name, width=14, wall=False, trail=False, broken=False, stale=False, normal=False):
    grid = np.full((80, 120), 100, dtype=int)
    lo, hi = 40-width//2, 40+width//2
    grid[lo:hi, 5:100] = 0
    grid[lo:hi, 100:] = -1
    if wall: grid[lo:hi, 50] = 100
    truth = grid.copy()
    if stale: truth[lo:hi, 50] = 0
    msg = OccupancyGrid()
    msg.info.resolution = .1
    msg.info.height, msg.info.width = grid.shape
    msg.data = grid.ravel().tolist()
    b = FeedbackWaypointBridge.__new__(FeedbackWaypointBridge)
    b.projected_map, b.robot_xy = msg, (1.05, 4.05)
    b.failed = []
    b.target_min_clearance, b.target_min_robot_distance = .4, .8
    b.target_search_radius = 5.
    b.local_obstacle_clearance = lambda x, y: (False, None)
    c = FrontierContinuity()
    if trail:
        c.trail = [np.array([x, 4.05]) for x in np.arange(1.05, 8.1, .5)]
    if broken: c.trail_breaks = {1}
    frontiers = {(9.95, (y+.5)*.1) for y in range(lo, hi)}

    def free(p):
        cell = b.map_cell(msg, *p)
        return cell is not None and grid[cell[1], cell[0]] == 0

    def clear(a, z):
        return all(free(p) for p in np.linspace(a, z, max(2, int(np.linalg.norm(np.array(z)-a)/.025)+1)))

    nodes = {}
    if normal:
        chain = [(x, 4.05) for x in (1.05, 3.05, 5.05, 7.05, 8.05)]
        nodes = {p: SimpleNamespace(neighbor_set=set(chain[max(0, i-1):i+2])-{p})
                 for i, p in enumerate(chain)}
    regions, _ = label(truth == 0)  # independent point-connectivity oracle, not SCAN
    robot_cell = b.map_cell(msg, *b.robot_xy)
    def assess(target):
        if target is None: return dict(target=None, endpoint_safe=None, point_connected=None)
        cell = b.map_cell(msg, *target)
        return dict(target=np.round(target, 3).tolist(),
                    endpoint_safe=bool(b.target_is_safe(msg, *target)),
                    point_connected=bool(regions[cell[1],cell[0]] != 0 and
                        regions[cell[1],cell[0]] == regions[robot_cell[1],robot_cell[0]]))

    begin = time.perf_counter()
    current = c.choose(None, frontiers, nodes, b.robot_xy, b.robot_xy, free, clear, set(), 0., 100)
    current_ms = (time.perf_counter()-begin)*1000
    begin = time.perf_counter()
    fallback = prototype(c, frontiers, b, grid)
    fallback_ms = (time.perf_counter()-begin)*1000
    a, z = assess(current), assess(fallback)
    a['ms'], z['ms'] = round(current_ms,2), round(fallback_ms,2)
    print(json.dumps(dict(case=name, current=a, simple_prototype=z), ensure_ascii=False))
    return a, z


if __name__ == '__main__':
    a, b = run('normal_graph', normal=True)
    assert a['point_connected'] and b['point_connected']
    a, b = run('open_corridor_no_graph_no_trail')
    assert a['target'] is None and b['endpoint_safe'] and b['point_connected']
    a, b = run('broken_history_open_corridor', trail=True, broken=True)
    assert a['target'] is None and b['point_connected']
    a, b = run('stale_wall_with_history', wall=True, trail=True, stale=True)
    assert a['point_connected'] and b['point_connected']
    a, b = run('real_closed_wall', wall=True)
    assert a['target'] is None and b['endpoint_safe'] and not b['point_connected']
    a, b = run('too_narrow_for_clearance', width=6)
    assert a['target'] is None and b['target'] is None
    print('PASS: six synthetic selection cases; not an end-to-end navigation test')
