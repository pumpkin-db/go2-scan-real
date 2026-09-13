#!/usr/bin/env python3
"""Convert FAST-LIO odometry to the pose message expected by elevation_mapping."""

import math

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class PoseAdapter:
    def __init__(self):
        in_topic = rospy.get_param('~odom_topic', '/LIO/odom_imu')
        out_topic = rospy.get_param('~pose_topic', '/elevation_mapping/pose')
        self.publisher = rospy.Publisher(out_topic, PoseWithCovarianceStamped,
                                         queue_size=20)
        rospy.Subscriber(in_topic, Odometry, self.callback, queue_size=30)
        rospy.loginfo('[go2_elevation] pose covariance: %s -> %s',
                      in_topic, out_topic)

    def callback(self, odometry):
        p = odometry.pose.pose.position
        q = odometry.pose.pose.orientation
        if not all(math.isfinite(value) for value in
                   (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            rospy.logwarn_throttle(2.0, '[go2_elevation] rejected non-finite odometry')
            return
        message = PoseWithCovarianceStamped()
        message.header = odometry.header
        message.pose = odometry.pose
        self.publisher.publish(message)


if __name__ == '__main__':
    rospy.init_node('fastlio_elevation_pose')
    PoseAdapter()
    rospy.spin()
