"""Feedback-driven frontier continuity; no execution/reach decisions live here."""
import heapq
import math
from collections import deque

import numpy as np
from scipy.spatial import cKDTree


class FrontierContinuity:
    def __init__(self, cluster_gap=0.5, match_distance=1.0,
                 no_progress_seconds=45.0, loop_radius=0.75):
        self.cluster_gap = cluster_gap
        self.match_distance = match_distance
        self.no_progress_seconds = no_progress_seconds
        self.loop_radius = loop_radius
        self.reset()

    def reset(self):
        self.active = None
        self.last_progress = 0.0
        self.best_distance = float('inf')
        self.known_cells = 0
        self.failures = 0
        self.history = deque(maxlen=12)
        self.cooldowns = []
        # Session-local record of the route the robot actually traversed.
        # It is a fallback graph only; SCAN still decides whether each leg is
        # currently safe to execute.
        self.trail = []
        self.trail_breaks = set()
        self.last_odom = None
        self.route = []
        self.route_uses_trail = False
        self._trail_edges = set()
        self.reason = 'idle'

    def feedback(self, xy, succeeded, now):
        if succeeded:
            self.history.append((np.asarray(xy, dtype=float), now))
            self.failures = 0
        else:
            self.failures += 1

    def penalty(self, xy, now):
        # Bounded soft preference, never masks the only route/backtracking option.
        return min(1.5, sum(0.5 * math.exp(-(now - t) / 60.0)
                   for p, t in self.history
                   if np.linalg.norm(np.asarray(xy) - p) < self.loop_radius))

    def observe(self, xy, stamp):
        """Sample the real odometry path without joining LIO jumps."""
        point = np.asarray(xy, dtype=float)
        if not np.isfinite(point).all():
            return
        if self.last_odom is not None:
            previous, previous_stamp = self.last_odom
            dt = stamp - previous_stamp
            if dt <= 0:
                if dt < -0.5:
                    if self.trail:
                        self.trail_breaks.add(len(self.trail))
                    self.trail.append(point.copy())
                    self.last_odom = (point.copy(), stamp)
                return
            # A timestamp gap or physically impossible displacement starts a
            # new segment instead of creating a false edge across the map.
            if dt > 1.0 or np.linalg.norm(point - previous) > max(0.75, 2.0 * dt):
                if self.trail:
                    self.trail_breaks.add(len(self.trail))
                self.trail.append(point.copy())
                self.last_odom = (point.copy(), stamp)
                return
        if not self.trail or np.linalg.norm(point - self.trail[-1]) >= 0.5:
            self.trail.append(point.copy())
        self.last_odom = (point.copy(), stamp)

    def clusters(self, frontiers):
        points = np.asarray(sorted(frontiers), dtype=float).reshape(-1, 2)
        if not len(points):
            return []
        tree = cKDTree(points)
        remaining = set(range(len(points)))
        result = []
        while remaining:
            seed = min(remaining)
            remaining.remove(seed)
            indices, queue = [seed], [seed]
            while queue:
                i = queue.pop()
                for j in tree.query_ball_point(points[i], self.cluster_gap):
                    if j in remaining:
                        remaining.remove(j)
                        queue.append(j)
                        indices.append(j)
            result.append(points[indices])
        return result

    def paths(self, nodes, start, free, clear):
        """Search the current free graph plus the route actually driven."""
        start = tuple(start)
        adjacency = {tuple(point): set() for point in nodes if free(point)}
        for point, node in nodes.items():
            point = tuple(point)
            if point not in adjacency:
                continue
            for neighbour in node.neighbor_set:
                neighbour = tuple(neighbour)
                if neighbour in adjacency and clear(point, neighbour):
                    adjacency[point].add(neighbour)
                    adjacency[neighbour].add(point)

        self._trail_edges = set()
        trail_keys = [tuple(np.round(point, 3)) for point in self.trail]
        for index, point in enumerate(trail_keys):
            adjacency.setdefault(point, set())
            if index and index not in self.trail_breaks:
                previous = trail_keys[index - 1]
                adjacency[point].add(previous)
                adjacency[previous].add(point)
                self._trail_edges.add(frozenset((point, previous)))

        # A trail may reconnect graph components only through currently clear
        # short links. The recorded trail edges themselves remain available
        # despite stale accumulated-map obstacles.
        ordinary = list(nodes)
        if ordinary and trail_keys:
            tree = cKDTree(np.asarray(ordinary))
            for point in trail_keys:
                for index in tree.query_ball_point(point, 1.5):
                    neighbour = tuple(ordinary[index])
                    if neighbour in adjacency and clear(point, neighbour):
                        adjacency[point].add(neighbour)
                        adjacency[neighbour].add(point)

        adjacency.setdefault(start, set())
        if trail_keys:
            nearest = min(trail_keys, key=lambda point: math.hypot(
                point[0] - start[0], point[1] - start[1]))
            if math.hypot(nearest[0] - start[0], nearest[1] - start[1]) <= 0.8:
                adjacency[start].add(nearest)
                adjacency[nearest].add(start)
                self._trail_edges.add(frozenset((start, nearest)))
        for point in ordinary:
            point = tuple(point)
            if point in adjacency and math.hypot(
                    point[0] - start[0], point[1] - start[1]) <= 2.0 \
                    and clear(start, point):
                adjacency[start].add(point)
                adjacency[point].add(start)

        distances, parents, queue = {start: 0.0}, {start: None}, [(0.0, start)]
        while queue:
            distance, u = heapq.heappop(queue)
            if distance != distances[u]:
                continue
            for v in sorted(adjacency[u]):
                candidate = distance + math.hypot(v[0] - u[0], v[1] - u[1])
                if candidate < distances.get(v, float('inf')):
                    distances[v], parents[v] = candidate, u
                    heapq.heappush(queue, (candidate, v))
        return distances, parents

    def choose(self, proposed, frontiers, nodes, start, robot, free, clear,
               excluded, now, known_cells, observation_range=2.5,
               waypoint_range=5.0, min_distance=0.8):
        """Return a graph waypoint, or None when no checked route exists.

        Called only when no goal is pending. Holds a reachable frontier component,
        releases on disappearance/failure/no progress, and retries all components
        if soft cooldown would otherwise suppress every choice.
        """
        self.route = []
        self.route_uses_trail = False
        regions = self.clusters(frontiers)
        if not regions:
            self.active = None
            self.reason = 'no_frontiers'
            return proposed
        distances, parents = self.paths(nodes, start, free, clear)
        robot = np.asarray(robot)
        excluded = set(excluded)
        trees = [cKDTree(r) for r in regions]
        routes = {}
        for index, tree in enumerate(trees):
            choices = []
            for p, distance in distances.items():
                if p in excluded or np.linalg.norm(np.asarray(p) - robot) <= min_distance:
                    continue
                if not free(p):
                    continue
                near = tree.query_ball_point(p, observation_range)
                near.sort(key=lambda j: np.linalg.norm(np.asarray(p) - regions[index][j]))
                visible = next((j for j in near if clear(p, regions[index][j])), None)
                if visible is None:
                    continue
                offset = np.linalg.norm(np.asarray(p) - regions[index][visible])
                choices.append((distance + offset + self.penalty(p, now), p))
            if choices:
                routes[index] = min(choices)[1]
        history_approach = not routes and proposed is None
        if history_approach:
            # A return leg need not already observe the frontier. Reposition
            # along recorded, reachable history first; SCAN checks each leg.
            # Require progress toward the frontier so reaching the closest
            # historical point cannot create an endless back-and-forth loop.
            trail_points = {tuple(np.round(p, 3)) for p in self.trail}
            for index, tree in enumerate(trees):
                current_distance = float(tree.query(robot)[0])
                choices = []
                for p in trail_points.intersection(distances):
                    if p in excluded or not free(p) or np.linalg.norm(
                            np.asarray(p) - robot) <= min_distance:
                        continue
                    offset = float(tree.query(p)[0])
                    if offset + min_distance < current_distance:
                        choices.append((offset, distances[p], p))
                if choices:
                    routes[index] = min(choices)[2]
        if not routes:
            self.active = None
            self.reason = 'frontiers_no_reachable_approach'
            return proposed

        active_index = None
        if self.active is not None:
            old_tree = cKDTree(self.active)
            overlap = [int(np.sum(old_tree.query(r)[0] <= self.match_distance)) for r in regions]
            if max(overlap):
                active_index = int(np.argmax(overlap))
            if active_index is not None:
                distance = float(trees[active_index].query(robot)[0])
                if known_cells >= self.known_cells + 20 or distance < self.best_distance - 0.5:
                    self.last_progress = now
                    self.known_cells = known_cells
                    self.best_distance = distance
            reason = None
            if active_index not in routes:
                reason = 'region_exhausted_or_unreachable'
            elif self.failures >= 2:
                reason = 'region_two_failures'
            elif now - self.last_progress >= self.no_progress_seconds:
                reason = 'region_no_progress'
            if reason:
                if active_index is not None:
                    self.cooldowns.append((regions[active_index].copy(), now + 60.0))
                self.active = None
                active_index = None
                self.reason = reason

        if active_index is None:
            self.cooldowns = [(r, until) for r, until in self.cooldowns if until > now]
            available = [i for i in routes if not any(
                np.min(cKDTree(r).query(regions[i])[0]) <= self.match_distance
                for r, _ in self.cooldowns)]
            if not available:
                available = list(routes)  # cooldown must never cause a stall
            reference = np.asarray(proposed) if proposed is not None else robot
            active_index = min(available, key=lambda i: (
                float(trees[i].query(reference)[0]) if proposed is not None else
                distances[routes[i]] + float(trees[i].query(routes[i])[0])))
            self.last_progress = now
            self.best_distance = float(trees[active_index].query(robot)[0])
            self.known_cells = known_cells
            self.failures = 0
            self.reason += ':select_region'
        else:
            self.reason = 'hold_region'
        self.active = regions[active_index].copy()
        if history_approach:
            self.reason += ':history_approach'

        # Keep the policy choice if it observes this region and is reachable.
        if proposed is not None:
            p = tuple(proposed)
            if p in distances and p not in excluded and np.linalg.norm(np.asarray(p)-robot) > min_distance:
                nearby = trees[active_index].query_ball_point(p, observation_range)
                if any(clear(p, self.active[j]) for j in nearby):
                    self.reason += ':policy'
                    return np.asarray(p)

        end = routes[active_index]
        path = []
        p = end
        while parents[p] is not None:
            path.append(p)
            p = parents[p]
        path.reverse()
        edges = [frozenset((a, b)) for a, b in zip([tuple(start)] + path, path)]
        self.route_uses_trail = any(edge in self._trail_edges for edge in edges)
        self.route = [tuple(start)] + path

        if self.route_uses_trail:
            # Follow the recorded polyline in short legs. Stop at meaningful
            # corners instead of asking SCAN to cut straight through a wall.
            leg = []
            travelled = 0.0
            previous = tuple(robot)
            for index, point in enumerate(path):
                travelled += np.linalg.norm(np.asarray(point) - np.asarray(previous))
                if travelled > waypoint_range:
                    break
                leg.append(point)
                previous = point
                if index + 1 < len(path) and np.linalg.norm(
                        np.asarray(point) - robot) > min_distance:
                    incoming = robot if index == 0 else np.asarray(path[index - 1])
                    first = np.asarray(point) - incoming
                    second = np.asarray(path[index + 1]) - np.asarray(point)
                    if np.dot(first, second) < math.cos(math.radians(30)) \
                            * np.linalg.norm(first) * np.linalg.norm(second):
                        break
            candidates = [point for point in leg if point not in excluded and
                          np.linalg.norm(np.asarray(point) - robot) > min_distance and
                          free(point)]
            if not candidates:
                self.reason += ':return_route_no_waypoint'
                return proposed
            self.reason += ':return_route'
            return np.asarray(candidates[-1])

        # Farthest visible path point within normal waypoint range. The bridge
        # still owns clearance relocation and SCAN still owns all arrival tests.
        candidates = [p for p in path if p not in excluded and
                      min_distance < np.linalg.norm(np.asarray(p)-robot) <= waypoint_range
                      and clear(robot, p)]
        if not candidates:
            self.reason += ':no_visible_waypoint'
            return proposed
        self.reason += ':graph_route'
        return np.asarray(candidates[-1])
