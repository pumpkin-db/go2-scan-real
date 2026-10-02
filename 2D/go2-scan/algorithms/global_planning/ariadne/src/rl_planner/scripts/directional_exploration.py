"""Optional direction preference. None always hands the decision back to RL.

Uses the existing graph/travelled-route search, but none of its region cooldowns
or failure counters. Execution and endpoint clearance remain in the SCAN chain.
"""
import numpy as np
from scipy.spatial import cKDTree


class DirectionalExploration:
    def __init__(self):
        self.reset()

    def reset(self):
        self.heading = None
        self.departure = None

    def dispatched(self, robot):
        self.departure = np.asarray(robot, dtype=float).copy()

    def feedback(self, succeeded, robot, trail):
        if not succeeded or self.departure is None:
            return
        robot = np.asarray(robot, dtype=float)
        displacement = robot - self.departure
        if np.linalg.norm(displacement) < 0.3:
            return  # Arrival tolerance is not evidence of a new travel direction.
        # Use the recent actual route so a corridor bend changes our preference.
        if trail and np.linalg.norm(robot - trail[-1]) < 0.75:
            for point in reversed(trail):
                if np.linalg.norm(robot - point) >= 1.0:
                    displacement = robot - point
                    break
        self.heading = displacement / np.linalg.norm(displacement)

    def choose(self, frontiers, nodes, start, robot, free, clear, excluded,
               route_search, observation_range, waypoint_range=5.0,
               min_distance=0.8):
        if self.heading is None:
            return None
        if not len(frontiers):
            self.heading = None
            return None
        robot = np.asarray(robot, dtype=float)
        frontier_points = np.asarray(sorted(frontiers), dtype=float)
        tree = cKDTree(frontier_points)
        distances, parents = route_search.paths(nodes, start, free, clear)
        candidates = []
        for end, distance in distances.items():
            if end in excluded or not free(end):
                continue
            nearby = tree.query_ball_point(end, observation_range)
            if not any(clear(end, frontier_points[i]) for i in nearby):
                continue
            path = []
            point = end
            while parents[point] is not None:
                path.append(point)
                point = parents[point]
            path.reverse()
            if not path:
                continue

            # Assess the route's departure, not the straight bearing to a distant
            # frontier: a U-shaped corridor may end behind the robot.
            departure = next((np.asarray(p) - robot for p in path
                              if np.linalg.norm(np.asarray(p) - robot) >= 1.5),
                             np.asarray(path[-1]) - robot)
            length = np.linalg.norm(departure)
            if length <= min_distance:
                continue
            alignment = float(np.dot(departure / length, self.heading))
            if alignment < -0.5:
                continue  # Backtracking remains available through the RL fallback.

            leg, travelled, previous = [], 0.0, robot
            for point in path:
                travelled += np.linalg.norm(np.asarray(point) - previous)
                previous = np.asarray(point)
                if travelled > waypoint_range + 1e-6:
                    break
                if (point not in excluded and free(point)
                        and min_distance < np.linalg.norm(previous - robot) <= waypoint_range
                        and clear(robot, point)):
                    leg.append(point)
            if not leg:
                continue
            waypoint = np.asarray(leg[-1])
            # Forward before sideways; among comparable routes favour advancing
            # farther. Frontier population never outweighs the chosen direction.
            rank = (0 if alignment >= 0.5 else 1,
                    -float(np.dot(waypoint - robot, self.heading)),
                    -float(np.linalg.norm(waypoint - robot)), distance, end)
            candidates.append((rank, waypoint))
        if candidates:
            return min(candidates, key=lambda item: item[0])[1]
        # No extra waiting, retry budget, region quarantine or completion state.
        self.heading = None
        return None
