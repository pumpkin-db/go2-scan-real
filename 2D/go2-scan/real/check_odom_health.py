#!/usr/bin/env python3
"""在允许真机运动前检查 LIO 频率、新鲜度和时间轴推进速度。"""

import math
import sys
import threading
import time

import rospy
from nav_msgs.msg import Odometry


def main():
    rospy.init_node('check_pointlio_odom_health', anonymous=True, disable_signals=True)
    topic = rospy.get_param('~topic', '/pointlio/Odometry')
    duration = float(rospy.get_param('~duration', 5.0))
    min_rate = float(rospy.get_param('~min_rate', 8.0))
    max_gap_limit = float(rospy.get_param('~max_gap', 0.2))
    max_age_limit = float(rospy.get_param('~max_age', 0.5))
    max_future_limit = float(rospy.get_param('~max_future', 0.05))
    min_stamp_progress = float(rospy.get_param('~min_stamp_progress', 0.9))

    lock = threading.Lock()
    samples = []

    def callback(msg):
        receive_wall = time.monotonic()
        receive_ros = rospy.Time.now().to_sec()
        with lock:
            samples.append((receive_wall, receive_ros, msg.header.stamp.to_sec()))

    sub = rospy.Subscriber(topic, Odometry, callback, queue_size=50,
                           tcp_nodelay=True)
    started = time.monotonic()
    deadline = started + duration
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        time.sleep(0.05)
    sub.unregister()

    with lock:
        samples = list(samples)
    elapsed = max(time.monotonic() - started, 1e-6)
    rate = len(samples) / elapsed
    receive_times = [sample[0] for sample in samples]
    stamps = [sample[2] for sample in samples]
    gaps = [b - a for a, b in zip(receive_times, receive_times[1:])]
    max_gap = max(gaps) if gaps else float('inf')
    stamps_valid = all(math.isfinite(stamp) and stamp > 0 for stamp in stamps)
    stamps_increase = stamps_valid and all(b > a for a, b in zip(stamps, stamps[1:]))
    ages = [receive_ros - stamp for _, receive_ros, stamp in samples] if stamps_valid else []
    max_age = max(ages) if ages else float('inf')
    max_future = max(0.0, max((-age for age in ages), default=float('inf')))
    receive_span = receive_times[-1] - receive_times[0] if len(samples) >= 2 else 0.0
    stamp_span = stamps[-1] - stamps[0] if len(samples) >= 2 and stamps_valid else 0.0
    stamp_progress = stamp_span / receive_span if receive_span > 0 else 0.0
    healthy = (len(samples) >= 2 and rate >= min_rate and max_gap <= max_gap_limit and
               stamps_increase and max_age <= max_age_limit and max_future <= max_future_limit and
               stamp_progress >= min_stamp_progress)
    print('[ODOM_HEALTH_CHECK] samples=%d rate=%.2fHz max_gap=%.3fs max_age=%.3fs '
          'max_future=%.3fs stamp_progress=%.3f limits=(%.2fHz, %.3fs, %.3fs, %.3fs, %.2f) result=%s' % (
        len(samples), rate, max_gap, max_age, max_future, stamp_progress,
        min_rate, max_gap_limit, max_age_limit, max_future_limit, min_stamp_progress,
        'PASS' if healthy else 'FAIL'))
    rospy.signal_shutdown('health check complete')
    return 0 if healthy else 1


if __name__ == '__main__':
    sys.exit(main())
