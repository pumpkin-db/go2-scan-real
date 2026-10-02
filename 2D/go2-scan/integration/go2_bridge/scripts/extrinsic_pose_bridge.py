#!/usr/bin/env python3
# 把 FAST-LIO /Odometry(map->base_footprint=IMU) 应用 base_footprint(IMU)->go2_body 外参，
# 得到 map->go2_body，发布 /quad_0/body_pose(nav_msgs/Odometry)。外参为已复核值，勿改。
import rospy, math
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix, quaternion_from_matrix, quaternion_from_euler, concatenate_matrices

# 复核通过：T_imu_body = base_footprint(IMU) -> go2_body
#  xyz=(0.10700,-0.02329,-0.21697)  RPY(0,-8deg,180deg)->(roll,pitch,yaw)=(0,-0.139626,3.141593)
Q_EXT = quaternion_from_euler(0.0, -8.0*math.pi/180.0, math.pi, 'sxyz')   # = (0.0698,0,0.9976,0)
T_XYZ = [0.10700, -0.02329, -0.21697]
def ext_matrix():
    m = quaternion_matrix(Q_EXT); m[:3,3] = T_XYZ; return m

rospy.init_node('extrinsic_pose_bridge', anonymous=True)
pub = rospy.Publisher('/quad_0/body_pose', Odometry, queue_size=10)
EXT = ext_matrix()

def cb(msg):
    p = msg.pose.pose
    T = quaternion_matrix([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
    T[:3,3] = [p.position.x, p.position.y, p.position.z]
    R = concatenate_matrices(T, EXT)   # map->go2_body
    out = Odometry()
    out.header = msg.header; out.child_frame_id = "go2_body"
    out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z = R[0,3],R[1,3],R[2,3]
    q = quaternion_from_matrix(R)
    out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w = q[0],q[1],q[2],q[3]
    out.pose.covariance = msg.pose.covariance
    out.twist = msg.twist
    pub.publish(out)

rospy.Subscriber('/Odometry', Odometry, cb, queue_size=1)
rospy.logwarn('extrinsic_pose_bridge: /quad_0/body_pose (map->go2_body, extrinsic verified)')
rospy.spin()
