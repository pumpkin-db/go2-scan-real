"""Offline goal orchestration tests: no ROS master or robot required."""
import unittest
from unittest.mock import patch
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PointStamped
from scan_planner.msg import GoalFeedback
import ariadne_goal_bridge as mod


class Publisher:
    def __init__(self, *args, **kwargs): self.messages = []
    def publish(self, value): self.messages.append(value)
    def get_num_connections(self): return 1


class FeedbackTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.patches = [patch.object(mod.time, 'monotonic', lambda: self.now),
                        patch.object(rospy.Time, 'now', lambda: rospy.Time.from_sec(self.now)),
                        patch.object(rospy, 'get_param', lambda k,d=None: True if k == '/ariadne/execution_ready' else d),
                        patch.object(rospy, 'Publisher', Publisher)]
        for name in ('Subscriber','Timer','on_shutdown','loginfo','logwarn','logwarn_throttle','logerr_throttle'):
            self.patches.append(patch.object(rospy, name, lambda *a,**kw: None))
        for p in self.patches: p.start()
        self.addCleanup(lambda: [p.stop() for p in reversed(self.patches)])
        self.b = mod.FeedbackWaypointBridge()
        self.b.motion = True
        grid = OccupancyGrid()
        grid.info.resolution = .1
        grid.info.width = grid.info.height = 121
        grid.info.origin.position.x = grid.info.origin.position.y = -6
        grid.data = [0]*(121*121)
        self.b.map_cb(grid)
        self.b.robot_xy = (0,0)
        m = PointStamped()
        m.header.stamp = rospy.Time(80)
        m.point.x = 2
        self.b.waypoint_cb(m)
        self.advance(0)

    def advance(self, dt, v=0, w=0, gate='READY'):
        self.now += dt
        self.b.command = (self.now,v,w)
        self.b.gate = (self.now,gate)
        self.b.odom_received = self.now
        self.b.last_feedback_received = self.now
        self.b.tick(None)

    def test_timeout_six_seconds(self):
        for _ in range(59): self.advance(.1)
        self.assertEqual(self.b.counts['no_command'],0)
        self.advance(.11)
        self.assertEqual(self.b.counts['no_command'],1)
        self.assertIsNone(self.b.decision)
        self.assertEqual(self.b.result.reason, 'NO_VALID_CMD')

    def test_turning_does_not_timeout(self):
        for _ in range(100): self.advance(.1,w=.3)
        self.assertEqual(self.b.counts['no_command'],0)

    def test_turning_without_xy_progress_eventually_fails(self):
        for _ in range(201): self.advance(.1,w=.3)
        self.assertIsNone(self.b.decision)
        self.assertEqual(self.b.result.reason,'NO_PROGRESS')

    def test_active_feedback_maps_id_and_effective_goal(self):
        r=GoalFeedback(); r.state=r.ACTIVE; r.distance_xy=1.4
        r.effective_goal.x=1.8; r.request_header=self.b.attempt.header
        self.b.events.append(('feedback',r)); self.advance(.1,v=.1)
        mapped=self.b.result_pub.messages[-1]
        self.assertEqual(mapped.request_header.stamp,rospy.Time(80))
        self.assertEqual(mapped.effective_goal.x,1.8)

    def test_locked_does_not_replace(self):
        for _ in range(100): self.advance(.1,gate='LOCKED')
        self.assertEqual(len(self.b.path_pub.messages),1)

    def test_motion_false(self):
        self.b.motion=False
        for _ in range(100): self.advance(.1)
        self.assertEqual(self.b.counts['no_command'],0)

    def test_plan_failure_returns_to_ar(self):
        r=GoalFeedback(); r.state=r.FAILED
        r.reason='LOCAL_PLAN_TIMEOUT'
        r.request_header=self.b.attempt.header
        self.b.events.append(('feedback',r))
        self.advance(.1,gate='WAIT_TRAJECTORY')
        self.assertIsNone(self.b.decision)
        self.assertEqual(self.b.result.reason,'LOCAL_PLAN_TIMEOUT')

    def test_single_pulses_do_not_hide_timeout(self):
        for i in range(61): self.advance(.1,v=.1 if i%20==0 else 0)
        self.assertEqual(self.b.counts['no_command'],1)

    def test_late_result_and_matching_success(self):
        r=GoalFeedback()
        r.state=r.SUCCEEDED
        r.request_header.stamp=rospy.Time(1)
        self.b.events.append(('feedback',r))
        self.advance(.1)
        self.assertIsNotNone(self.b.decision)
        r.request_header=self.b.attempt.header
        self.b.events.append(('feedback',r))
        self.advance(.1)
        self.assertIsNone(self.b.decision)
        self.assertEqual(self.b.result.request_header.stamp,rospy.Time(80))

    def test_bridge_never_relocates_or_retries(self):
        self.assertEqual(self.b.current,(2,0))
        r=GoalFeedback(); r.state=r.FAILED; r.reason='GOAL_BLOCKED'
        r.request_header=self.b.attempt.header
        self.b.events.append(('feedback',r)); self.advance(.1)
        self.assertEqual(len(self.b.path_pub.messages),1)
        self.assertEqual(self.b.counts['forwarded'],1)

    def test_clearance_boundary(self):
        grid=self.b.projected_map
        data = list(grid.data)
        data[60 * grid.info.width + 60] = 100
        grid.data = data
        # Occupied cell x extent is [0.0, 0.1]. Clearance is strict >0.4m.
        self.assertFalse(self.b.target_is_safe(grid, .5, .05))
        self.assertTrue(self.b.target_is_safe(grid, .501, .05))

if __name__=='__main__': unittest.main()
