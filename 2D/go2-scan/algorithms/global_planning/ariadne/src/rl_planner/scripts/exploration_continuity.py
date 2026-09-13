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

    @staticmethod
    def paths(nodes, start, free, clear):
        """One heap search, independent of utility and rarefied/key nodes."""
        start = tuple(start)
        if start not in nodes or not free(start):
            return {}, {}
        distances, parents, queue = {start: 0.0}, {start: None}, [(0.0, start)]
        checked = {}
        while queue:
            distance, u = heapq.heappop(queue)
            if distance != distances[u]:
                continue
            for v in sorted(nodes[u].neighbor_set):
                v = tuple(v)
                if v == u or v not in nodes:
                    continue
                edge = tuple(sorted((u, v)))
                if edge not in checked:
                    checked[edge] = free(v) and free(u) and clear(u, v)
                if not checked[edge]:
                    continue
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
                near = tree.query_ball_point(p, observation_range)
                near.sort(key=lambda j: np.linalg.norm(np.asarray(p) - regions[index][j]))
                visible = next((j for j in near if clear(p, regions[index][j])), None)
                if visible is None:
                    continue
                offset = np.linalg.norm(np.asarray(p) - regions[index][visible])
                choices.append((distance + offset + self.penalty(p, now), p))
            if choices:
                routes[index] = min(choices)[1]
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
