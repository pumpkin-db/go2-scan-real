"""Run in AR Python environment, with ROS/SCAN workspace sourced."""
import sys
import os
from pathlib import Path
from collections import deque
from unittest.mock import patch
from types import SimpleNamespace
root=Path(__file__).resolve().parents[3]
cache=root/'.agent-cache'/'matplotlib'
cache.mkdir(parents=True,exist_ok=True)
os.environ['MPLCONFIGDIR']=str(cache)
sys.path.insert(0,str(root/'algorithms/global_planning/ariadne/src/rl_planner/scripts'))
import rl_planner as m
from geometry_msgs.msg import PointStamped
from scan_planner.msg import GoalFeedback

class Publisher:
    def __init__(self): self.messages=[]
    def publish(self,m): self.messages.append(m)

r=m.Runner.__new__(m.Runner)
r.goal_feedback_enabled=True
r.pending_goal=PointStamped()
r.pending_goal.header.stamp=m.rospy.Time(123)
r.pending_goal.point.x=2
r.goal_results=deque()
r.odom_samples=deque()
r.goal_accepted=False
r.continuity=SimpleNamespace(
    observe=lambda *a:None, feedback=lambda *a:None,
    frontier_classes=lambda frontiers: (
        m.np.asarray(list(frontiers), dtype=float).reshape(-1, 2),
        m.np.empty((0, 2)), m.np.empty((0, 2))))
r.reset_requested=False
r.paused=False
r.ground_z_ref=0.0
r.failed_until={}
r.waypoint_pub=Publisher()
r.start=None
r.publish_status=lambda:None
with patch.object(m.rospy,'get_param',lambda k,d=None:True):
    # No robot/model fields exist: executing a policy here would fail this test.
    r.run()
    assert r.pending_goal is not None and len(r.waypoint_pub.messages)==1
    old=GoalFeedback(); old.request_header.stamp=m.rospy.Time(122); old.state=old.SUCCEEDED
    r.goal_results.append(old); r.run()
    assert r.pending_goal is not None
    current=GoalFeedback(); current.request_header.stamp=m.rospy.Time(123); current.state=current.FAILED
    r.goal_results.append(current); r.run()
    assert r.pending_goal is None and (2,0) in r.failed_until
print('PASS: pending goal skips RL; stale result ignored; matching failure releases decision')

# Exercise the real graph-refresh and visualization methods, not a stubbed run().
for motion in (False, True):
    r.pending_goal=PointStamped()
    r.pending_goal.header.stamp=m.rospy.Time(123)
    r.start=m.np.array([0.,0.])
    r.robot_location=r.start.copy()
    r.map_info=object()
    r.publish_graph=True
    r.frontier_pub=Publisher(); r.node_pub=Publisher(); r.edge_pub=Publisher()
    calls=[]
    def update(map_info, location):
        calls.append(map_info)
        r.robot.frontier={(float(len(calls)), 1.)}
    r.robot=SimpleNamespace(
        node_manager=SimpleNamespace(check_valid_node=lambda *a: None),
        update_planning_state=update,
        key_node_coords=[], key_utility=[], frontier=set())
    with patch.object(m.rospy,'get_param',lambda k,d=None:True), \
         patch.object(m.rospy.Time,'now',return_value=m.rospy.Time(200)):
        r.run()
        r.map_info=object()
        r.run()
    assert len(calls)==2 and calls[-1] is r.map_info
    assert len(r.frontier_pub.messages)==len(r.node_pub.messages)==len(r.edge_pub.messages)==2
    assert r.frontier_pub.messages[0].data != r.frontier_pub.messages[1].data
    assert r.pending_goal.header.stamp==m.rospy.Time(123)
    # No select_next_waypoint method exists: inference while waiting would fail.
print('PASS: waiting refreshes changed frontier/graph every tick, preserves goal ID, never runs policy; shared motion path')
