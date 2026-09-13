#!/usr/bin/env python3
# go2_bridge: octomap /projected_map_occ -> /projected_map，动态设置 2D 网格地面 z。
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64
def main():
    rospy.init_node('projected_map_ground')
    ground_z = rospy.get_param('~ground_z', -0.25)
    in_t  = rospy.get_param('~in_topic', '/projected_map_occ')
    out_t = rospy.get_param('~out_topic', '/projected_map')
    pub = rospy.Publisher(out_t, OccupancyGrid, queue_size=1)
    state = {'ground_z': ground_z, 'last_map': None}
    def floor_cb(msg):
        state['ground_z'] = msg.data
        if state['last_map'] is not None:
            state['last_map'].info.origin.position.z = state['ground_z']
            pub.publish(state['last_map'])
    def cb(msg):
        msg.info.origin.position.z = state['ground_z']
        state['last_map'] = msg
        pub.publish(msg)
    rospy.Subscriber('/floor_context/z_ref', Float64, floor_cb, queue_size=1)
    rospy.Subscriber(in_t, OccupancyGrid, cb, queue_size=1)
    rospy.loginfo('[projected_map_ground] %s -> %s, initial origin.z=%.3f; following /floor_context/z_ref',
                  in_t, out_t, ground_z)
    rospy.spin()
if __name__ == '__main__':
    main()
