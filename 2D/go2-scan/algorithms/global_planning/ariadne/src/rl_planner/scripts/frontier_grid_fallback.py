"""Stateless last-resort frontier approach. SCAN retains execution authority."""
import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree
from skimage.graph import MCP_Geometric


def grid_frontier_waypoint(grid, origin, resolution, robot, regions, excluded=(),
                           clearance=.4, min_distance=.8, max_leg=5.):
    """Return (waypoint, full free-cell route), or (None, []).

    OccupancyGrid convention: origin is the cell corner, output at cell centres.
    Only known FREE (0) is traversable. Four-neighbour paths cannot cut corners.
    Endpoint clearance is conservative distance to occupied cell boundaries;
    this is not a swept-body collision checker or a replacement for SCAN.
    """
    grid = np.asarray(grid)
    origin, robot = np.asarray(origin), np.asarray(robot)
    if resolution <= 0 or not regions or not np.isfinite(robot).all():
        return None, []
    start = np.floor((robot-origin)/resolution).astype(int)
    h, w = grid.shape
    if not (0 <= start[0] < w and 0 <= start[1] < h):
        return None, []
    free = grid == 0
    if not free[start[1], start[0]]:
        return None, []  # never treat occupied/unknown background as a component
    occupied = grid >= 50
    safe = free.copy()
    travel_cost = np.ones(grid.shape)
    if occupied.any():
        boundary_distance = distance_transform_edt(~occupied) * resolution \
            - resolution / np.sqrt(2.)
        safe &= boundary_distance > clearance + 1e-6
        # A pure shortest path hugs door jambs and may have no safe intermediate
        # waypoint. Prefer open space without making clearance a hard route mask.
        travel_cost += clearance / np.maximum(boundary_distance, resolution / 2.)
    search = MCP_Geometric(np.where(free, travel_cost, np.inf), fully_connected=False)
    costs, _ = search.find_costs([(start[1], start[0])])
    ys, xs = np.nonzero(safe & np.isfinite(costs))
    points = origin + (np.column_stack((xs, ys)) + .5) * resolution
    valid = np.linalg.norm(points-robot, axis=1) > min_distance
    for p in excluded:
        valid &= np.linalg.norm(points-np.asarray(p), axis=1) >= .6
    ys, xs, points = ys[valid], xs[valid], points[valid]
    if not len(points):
        return None, []

    def visible(point):
        samples = np.linspace(robot, point, max(2, int(np.linalg.norm(point-robot)
                                                       / resolution * 4) + 2))
        cells = np.floor((samples-origin)/resolution).astype(int)
        if not free[cells[:, 1], cells[:, 0]].all():
            return False
        a, b = cells[:-1], cells[1:]
        # When crossing a corner, both adjacent cells must also be free.
        return bool(free[a[:, 1], b[:, 0]].all() and free[b[:, 1], a[:, 0]].all())

    ordered = sorted(regions, key=lambda r: (
        int(np.min(np.linalg.norm(np.asarray(r)-robot, axis=1))), -len(r)))
    for region in ordered:
        distance = cKDTree(region).query(points)[0]
        near = np.flatnonzero(distance <= .8)
        if not len(near):
            continue
        # Closest reachable approach by route length, not Euclidean wall crossing.
        index = min(near, key=lambda i: (costs[ys[i], xs[i]], distance[i]))
        cells = np.asarray(search.traceback((ys[index], xs[index])))
        path = origin + (cells[:, ::-1] + .5) * resolution
        candidates = []
        for i, point in enumerate(path[1:], 1):
            if i * resolution > max_leg:
                break
            y, x = cells[i]
            if safe[y, x] and np.linalg.norm(point-robot) > min_distance \
                    and all(np.linalg.norm(point-np.asarray(p)) >= .6 for p in excluded) \
                    and visible(point):
                candidates.append(point)
        if candidates:
            return candidates[-1], path
    return None, []
