#!/usr/bin/env python3
# go2_bridge: octomap /projected_map_occ -> /projected_map, 把 2D 网格 origin.z 平移到地面 z(此处-0.25)
import rospy
from nav_msgs.msg import OccupancyGrid
def main():
    rospy.init_node('projected_map_ground')
    ground_z = rospy.get_param('~ground_z', -0.25)
    in_t  = rospy.get_param('~in_topic', '/projected_map_occ')
    out_t = rospy.get_param('~out_topic', '/projected_map')
    pub = rospy.Publisher(out_t, OccupancyGrid, queue_size=1)
    def cb(msg):
        msg.info.origin.position.z = ground_z
        pub.publish(msg)
    rospy.Subscriber(in_t, OccupancyGrid, cb, queue_size=1)
    rospy.loginfo('[projected_map_ground] %s -> %s, origin.z=%.3f', in_t, out_t, ground_z)
    rospy.spin()
if __name__ == '__main__':
    main()
