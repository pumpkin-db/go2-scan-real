#!/usr/bin/env python3
"""Supply world scans and LiDAR poses at identical stamps; do not lock or alter Z."""
import copy
import message_filters
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

class Inputs:
    def __init__(self):
        self.period = 1.0 / float(rospy.get_param('~frequency', 5.0))
        self.last = None
        self.cloud_pub = rospy.Publisher('/tare/registered_scan', PointCloud2, queue_size=2)
        self.pose_pub = rospy.Publisher('/tare/state_estimation', Odometry, queue_size=2)
        self.scan_pose_pub = rospy.Publisher('/tare/state_estimation_at_scan', Odometry, queue_size=2)
        self.cloud_sub = message_filters.Subscriber('/mid360_points', PointCloud2, queue_size=6)
        self.pose_sub = message_filters.Subscriber('/quad_0/lidar_pose', Odometry, queue_size=30)
        self.sync = message_filters.TimeSynchronizer([self.cloud_sub, self.pose_sub], 50)
        self.sync.registerCallback(self.pair)

    def pair(self, cloud, pose):
        if cloud.header.frame_id != 'map' or pose.header.frame_id != 'map':
            rospy.logerr_throttle(5, '[TARE_INPUT] expected map world cloud and LiDAR pose')
            return
        stamp = cloud.header.stamp
        if self.last is not None and (stamp - self.last).to_sec() < self.period - 0.001:
            return
        self.last = stamp
        self.pose_pub.publish(pose)
        self.scan_pose_pub.publish(pose)
        self.cloud_pub.publish(cloud)
        rospy.loginfo_throttle(10, '[TARE_INPUT] same-stamp world cloud/LiDAR pose at <=5Hz; Z unchanged')

if __name__ == '__main__':
    rospy.init_node('tare_input_bridge')
    Inputs()
    rospy.spin()
