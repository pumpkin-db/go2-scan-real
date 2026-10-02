#!/usr/bin/env python3
"""Forward the freshest TARE intent to SCAN only when SCAN is ready."""
import copy
import math
import threading

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from scan_planner.msg import GoalFeedback


class Goals:
    def __init__(self):
        self.pose = None
        self.pending = None
        self.last_id = 0
        self.active_id = None
        self.lock = threading.RLock()
        self.min_goal_distance = float(rospy.get_param('~min_goal_distance', 0.6))
        self.use_path = bool(rospy.get_param('~use_path', False))
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.path_pub = rospy.Publisher('/initial_path', Path, queue_size=1) if self.use_path else None
        self.display_pub = rospy.Publisher('/way_point', PointStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber('/LIO/odom_vehicle', Odometry, self.odom, queue_size=5)
        self.goal_sub = rospy.Subscriber('/tare/execution_path' if self.use_path else '/tare/way_point',
                                         Path if self.use_path else PointStamped, self.goal, queue_size=1)
        self.feedback_sub = rospy.Subscriber('/scan/goal_feedback', GoalFeedback, self.feedback, queue_size=5)
        rospy.loginfo('[TARE -> SCAN] mode=%s; one active task, wait for SCAN result',
                      'reference-path' if self.use_path else 'single-goal')

    @staticmethod
    def remaining_path(msg, position):
        """Re-anchor the cached ordered segment after the preceding task finishes.

        Project onto its closest XY edge, discard only the already passed prefix,
        and preserve all remaining corners. SCAN checks the short connector.
        """
        best = None
        for i, (a, b) in enumerate(zip(msg.poses, msg.poses[1:])):
            a, b = a.pose.position, b.pose.position
            dx, dy = b.x - a.x, b.y - a.y
            length2 = dx * dx + dy * dy
            t = max(0.0, min(1.0, ((position.x - a.x) * dx +
                                    (position.y - a.y) * dy) / length2)) if length2 > 1e-12 else 0.0
            x, y = a.x + t * dx, a.y + t * dy
            d2 = (x - position.x) ** 2 + (y - position.y) ** 2
            # Prefer the later edge on a shared vertex to avoid replaying it.
            if best is None or d2 <= best[0]:
                best = (d2, i, x, y)
        result = Path()
        first = PoseStamped()
        first.pose.position = copy.deepcopy(position)
        first.pose.orientation.w = 1.0
        result.poses.append(first)
        _, i, x, y = best
        if math.hypot(x - position.x, y - position.y) > 1e-6:
            projection = copy.deepcopy(first)
            projection.pose.position.x, projection.pose.position.y = x, y
            result.poses.append(projection)
        for pose in msg.poses[i + 1:]:
            p, last = pose.pose.position, result.poses[-1].pose.position
            if math.hypot(p.x - last.x, p.y - last.y) > 1e-6:
                result.poses.append(copy.deepcopy(pose))
        return result

    def odom(self, msg):
        self.pose = msg

    def goal(self, msg):
        with self.lock:
            if self.active_id is not None:
                # TARE keeps updating its map and intent while SCAN executes;
                # retain only its latest intent, as the simulation bridge does.
                self.pending = copy.deepcopy(msg)
                return
            self._dispatch(msg)

    def _dispatch(self, msg):
        if not rospy.get_param('/exploration/execution_ready', False) or self.pose is None:
            rospy.logwarn_throttle(5, '[TARE -> SCAN] waiting for odometry/execution readiness')
            self.pending = copy.deepcopy(msg)
            return
        if msg.header.frame_id != 'map' or self.pose.header.frame_id != 'map':
            rospy.logerr_throttle(5, '[TARE -> SCAN] expected TARE and FAST-LIO odometry in map frame')
            return
        if self.use_path and len(msg.poses) < 2:
            rospy.logwarn_throttle(5, '[TARE -> SCAN] empty route; waiting for a planned path')
            return
        points = [pose.pose.position for pose in msg.poses] if self.use_path else [msg.point]
        p = points[-1]
        if not all(math.isfinite(v) for point in points for v in (point.x, point.y, point.z)):
            rospy.logwarn_throttle(5, '[TARE -> SCAN] ignored non-finite TARE waypoint')
            return

        pos = self.pose.pose.pose.position
        distance = math.hypot(p.x - pos.x, p.y - pos.y)
        if distance <= self.min_goal_distance:
            # SCAN would already regard this as reached; don't turn a no-op
            # lookahead fallback into a false successful exploration task.
            rospy.logwarn_throttle(5, '[TARE -> SCAN] no-op waypoint %.2fm from robot; waiting for a real path', distance)
            return

        self.last_id = max(rospy.Time.now().to_nsec(), self.last_id + 1)
        request_stamp = rospy.Time(self.last_id // 1000000000, self.last_id % 1000000000)
        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.header.stamp = request_stamp
        goal.pose.position.x = p.x
        goal.pose.position.y = p.y
        # Mode 1 treats this as an XY target and samples the current body Z.
        # Zero also passes SCAN's legacy negative-Z input guard.
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0

        shown = PointStamped()
        shown.header = copy.deepcopy(msg.header)
        shown.header.stamp = request_stamp
        shown.point = copy.deepcopy(p)
        route = None
        if self.use_path:
            route = self.remaining_path(msg, pos)
            if len(route.poses) < 2:
                return
            route.header = copy.deepcopy(goal.header)
            for pose in route.poses:
                pose.header = copy.deepcopy(route.header)
                # This is the fixed-height 2-D interface, not stair navigation.
                pose.pose.position.z = pos.z
        self.active_id = self.last_id
        if self.use_path:
            self.path_pub.publish(route)
        else:
            self.goal_pub.publish(goal)
        self.display_pub.publish(shown)
        rospy.loginfo('[TARE -> SCAN] goal=(%.2f, %.2f) distance=%.2fm id=%d; waiting for SCAN result',
                      p.x, p.y, distance, self.last_id)
        if self.use_path:
            rospy.loginfo('[TARE -> SCAN] reference-path poses=%d; endpoint matches this task', len(route.poses))

    def feedback(self, msg):
        with self.lock:
            if self.active_id is None or msg.request_header.stamp.to_nsec() != self.active_id:
                return
            if msg.state == GoalFeedback.ACTIVE:
                return
            if msg.state not in (GoalFeedback.SUCCEEDED, GoalFeedback.FAILED):
                return

            if msg.state == GoalFeedback.SUCCEEDED:
                rospy.loginfo('[TARE -> SCAN] REACHED distance=%.3fm', msg.distance_xy)
            else:
                rospy.logwarn('[TARE -> SCAN] FAILED reason=%s; handing control back to TARE', msg.reason)
            self.active_id = None

            latest = self.pending
            self.pending = None
            if latest is not None:
                self._dispatch(latest)


if __name__ == '__main__':
    rospy.init_node('tare_scan_bridge')
    Goals()
    rospy.spin()
