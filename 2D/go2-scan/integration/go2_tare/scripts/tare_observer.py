#!/usr/bin/env python3
"""Display-only latched snapshots and state labels; never publishes goals/control."""
import copy
import json
import time
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker, MarkerArray

class Observer:
    def __init__(self):
        self.pose = None
        self.goal = None
        self.finished = False
        self.report = None
        self.report_time = 0
        self.snapshots = {}
        self.pubs = {}
        self.subs = []
        self.markers = rospy.Publisher('/tare/status_markers', MarkerArray, queue_size=1, latch=True)
        for name, topic in [('frontiers','filtered_frontier_cloud'),('uncovered_frontiers','uncovered_frontier_cloud'),('viewpoints','viewpoint_vis_cloud'),('selected_viewpoints','selected_viewpoint_vis_cloud')]:
            self.pubs[name] = rospy.Publisher('/tare/display/'+name, PointCloud2, queue_size=1, latch=True)
            self.subs.append(rospy.Subscriber('/sensor_coverage_planner/'+topic, PointCloud2, self.cloud, callback_args=name, queue_size=1))
        self.subs += [rospy.Subscriber('/LIO/odom_vehicle',Odometry,self.odom,queue_size=1),
                      rospy.Subscriber('/tare/way_point',PointStamped,self.waypoint,queue_size=1),
                      rospy.Subscriber('/tare/diagnostics',String,self.diagnostic,queue_size=1),
                      rospy.Subscriber('/sensor_coverage_planner/exploration_finish',Bool,self.finish,queue_size=1)]
        rospy.Timer(rospy.Duration(1),self.draw)

    def cloud(self,msg,name):
        self.snapshots[name] = (msg.width*msg.height,time.monotonic())
        self.pubs[name].publish(msg)

    def odom(self,msg): self.pose=msg
    def waypoint(self,msg): self.goal=msg
    def finish(self,msg): self.finished=msg.data
    def diagnostic(self,msg):
        try: self.report=json.loads(msg.data);self.report_time=time.monotonic()
        except ValueError: return

    @staticmethod
    def marker(i,kind,frame='map'):
        m=Marker();m.header.frame_id=frame;m.header.stamp=rospy.Time.now();m.ns='tare_status';m.id=i;m.type=kind;m.action=Marker.ADD;m.pose.orientation.w=1.;m.color.a=1.;m.color.r=1.;m.color.g=1.;return m

    def draw(self,_):
        if self.pose is None: return
        status=self.marker(0,Marker.TEXT_VIEW_FACING,self.pose.header.frame_id)
        status.pose.position=copy.deepcopy(self.pose.pose.pose.position);status.pose.position.z+=1.5;status.scale.z=.16
        phase='FINISHED / RETURN HOME' if self.finished else 'EXPLORING'
        if self.report is None:
            counts='Waiting for decision diagnostics (restart new TARE binary)'
        else:
            d=self.report
            counts='candidates=%d  global_home=%d  local_done=%d' % (d['candidate_count'],d['global_return_home'],d['local_complete'])
            counts+='\nuncovered surfaces=%d  frontiers=%d%s' % (d['uncovered_surface_count'],d['uncovered_frontier_count'],'' if d['coverage_updated'] else ' [NOT UPDATED]')
            age=time.monotonic()-self.report_time
            if age>3: counts+='\nDIAGNOSTICS STALE %.0fs'%age
        status.text='TARE: '+phase+'\n'+counts
        if self.finished:status.text+='\nFrontier displays retain LAST received snapshot'
        markers=[status]
        if self.goal:
            label=self.marker(1,Marker.TEXT_VIEW_FACING,self.goal.header.frame_id);label.pose.position=copy.deepcopy(self.goal.point);label.pose.position.z+=.75;label.scale.z=.17
            label.text='TARE goal (%.2f, %.2f, %.2f)'%(self.goal.point.x,self.goal.point.y,self.goal.point.z)
            line=self.marker(2,Marker.LINE_LIST,self.goal.header.frame_id);line.scale.x=.025;line.points=[copy.deepcopy(self.goal.point),copy.deepcopy(label.pose.position)]
            markers.extend([label,line])
        self.markers.publish(MarkerArray(markers=markers))

if __name__=='__main__':
    rospy.init_node('tare_observer');Observer();rospy.spin()
