#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Move AR's OctoMap height band in stable, gravity-aligned 8cm steps."""

from collections import deque
import math
import statistics

import rospy
from dynamic_reconfigure.client import Client
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class HeightTracker(object):
    def __init__(self):
        self.odom_topic = rospy.get_param('~odom_topic', '/LIO/odom_imu')
        self.octomap_server = rospy.get_param('~octomap_server', '/octomap')
        self.initial_ground_z = float(rospy.get_param('~initial_ground_z', -0.310))
        self.initial_min_z = float(rospy.get_param('~initial_occupancy_min_z', -0.110))
        self.initial_max_z = float(rospy.get_param('~initial_occupancy_max_z', 0.490))
        self.step_height = float(rospy.get_param('~step_height', 0.08))
        self.initial_samples = int(rospy.get_param('~initial_samples', 20))
        self.filter_samples = int(rospy.get_param('~filter_samples', 9))
        self.initial_max_span = float(rospy.get_param('~initial_max_span', 0.03))
        self.max_abs_offset = float(rospy.get_param('~max_abs_offset', 5.0))
        if self.step_height <= 0.0 or self.initial_samples < 1 or self.filter_samples < 1:
            raise ValueError('sample counts and step_height must be positive')

        self.samples = deque(maxlen=max(self.initial_samples, self.filter_samples))
        self.baseline_z = None
        self.accepted_offset = 0.0
        self.floor_pub = rospy.Publisher('/floor_context/z_ref', Float64,
                                         queue_size=1, latch=True)
        self.client = Client(self.octomap_server, timeout=15.0)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)
        rospy.loginfo('[ar_height_tracker] waiting for %d stable samples on %s; step=%.2fm',
                      self.initial_samples, self.odom_topic, self.step_height)

    def apply_offset(self, new_offset):
        if abs(new_offset) > self.max_abs_offset:
            rospy.logerr_throttle(2.0, '[ar_height_tracker] reject offset %.3fm', new_offset)
            return False
        config = {
            'occupancy_min_z': self.initial_min_z + new_offset,
            'occupancy_max_z': self.initial_max_z + new_offset,
            'incremental_2D_projection': False,
        }
        try:
            self.client.update_configuration(config)
        except Exception as exc:
            rospy.logerr_throttle(2.0, '[ar_height_tracker] OctoMap update failed: %s', exc)
            return False
        old_offset = self.accepted_offset
        self.accepted_offset = new_offset
        floor_z = self.initial_ground_z + new_offset
        self.floor_pub.publish(Float64(floor_z))
        rospy.loginfo('[ar_height_tracker] height step %.3f -> %.3f; floor_z=%.3f',
                      old_offset, new_offset, floor_z)
        return True

    def odom_cb(self, msg):
        z = msg.pose.pose.position.z
        if not math.isfinite(z):
            return
        self.samples.append(z)

        if self.baseline_z is None:
            if len(self.samples) < self.initial_samples:
                return
            initial = list(self.samples)[-self.initial_samples:]
            span = max(initial) - min(initial)
            if span > self.initial_max_span:
                rospy.logwarn_throttle(
                    2.0, '[ar_height_tracker] initial Z is not stable (span=%.3fm)', span)
                return
            self.baseline_z = statistics.median(initial)
            self.floor_pub.publish(Float64(self.initial_ground_z))
            rospy.loginfo('[ar_height_tracker] baseline_z=%.3f floor_z=%.3f',
                          self.baseline_z, self.initial_ground_z)
            return

        filtered_z = statistics.median(list(self.samples)[-self.filter_samples:])
        measured_offset = filtered_z - self.baseline_z
        next_offset = self.accepted_offset
        tolerance = 1e-9
        while measured_offset - next_offset >= self.step_height - tolerance:
            next_offset += self.step_height
        while measured_offset - next_offset <= -self.step_height + tolerance:
            next_offset -= self.step_height
        if abs(next_offset - self.accepted_offset) > tolerance:
            self.apply_offset(next_offset)


def main():
    rospy.init_node('ar_height_tracker')
    HeightTracker()
    rospy.spin()


if __name__ == '__main__':
    main()
