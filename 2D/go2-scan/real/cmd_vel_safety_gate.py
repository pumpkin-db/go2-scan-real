#!/usr/bin/env python3
"""Fail-closed gate between the official SCAN controller and the Go2 bridge."""
import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scan_planner.msg import Bspline
from std_msgs.msg import String


class SafetyGate:
    def __init__(self):
        self.odom_timeout = float(rospy.get_param('~odom_timeout', 0.5))
        self.command_timeout = float(rospy.get_param('~command_timeout', 0.25))
        self.lock = threading.Lock()
        self.last_odom_wall = None
        self.last_odom_stamp = None
        self.last_command_wall = None
        self.command = Twist()
        self.unlocked = False
        self.ever_trajectory = False
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        self.status_pub = rospy.Publisher('/cmd_vel_gate/status', String, queue_size=2)
        rospy.Subscriber('/LIO/odom_vehicle', Odometry, self.on_odom, queue_size=2,
                         tcp_nodelay=True)
        rospy.Subscriber('/scan_planner/cmd_vel', Twist, self.on_command, queue_size=2,
                         tcp_nodelay=True)
        rospy.Subscriber('/planning/bspline', Bspline, self.on_trajectory, queue_size=2)
        rospy.Timer(rospy.Duration(0.02), self.on_timer)
        rospy.on_shutdown(self.on_shutdown)
        rospy.logwarn('cmd_vel safety gate locked until fresh odometry and a new SCAN trajectory')

    def odom_fresh(self, now):
        return self.last_odom_wall is not None and now - self.last_odom_wall <= self.odom_timeout

    def on_odom(self, message):
        stamp = message.header.stamp.to_sec()
        pose = message.pose.pose
        values = [stamp, pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        if not all(math.isfinite(value) for value in values) or stamp <= 0:
            return
        now = time.monotonic()
        with self.lock:
            if self.last_odom_stamp is not None and stamp <= self.last_odom_stamp:
                self.unlocked = False
                return
            self.last_odom_stamp = stamp
            self.last_odom_wall = now

    def on_command(self, message):
        values = [message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z]
        with self.lock:
            if not all(math.isfinite(value) for value in values):
                self.unlocked = False
                self.command = Twist()
                return
            self.command = message
            self.last_command_wall = time.monotonic()

    def on_trajectory(self, _message):
        now = time.monotonic()
        with self.lock:
            self.ever_trajectory = True
            if self.odom_fresh(now):
                self.unlocked = True
                rospy.logwarn('cmd_vel safety gate unlocked by a new trajectory')

    def on_timer(self, _event):
        now = time.monotonic()
        with self.lock:
            command_fresh = (self.last_command_wall is not None and
                             now - self.last_command_wall <= self.command_timeout)
            if not self.odom_fresh(now):
                self.unlocked = False
            output = self.command if self.unlocked and command_fresh else Twist()
            status = ('ODOM_STALE' if not self.odom_fresh(now) else
                      'COMMAND_STALE' if not command_fresh else
                      'READY' if self.unlocked else
                      'WAIT_TRAJECTORY' if not self.ever_trajectory else 'LOCKED')
        self.publisher.publish(output)
        self.status_pub.publish(String(status))

    def on_shutdown(self):
        stop = Twist()
        for _ in range(3):
            self.publisher.publish(stop)
            time.sleep(0.03)


if __name__ == '__main__':
    rospy.init_node('cmd_vel_safety_gate')
    SafetyGate()
    rospy.spin()
