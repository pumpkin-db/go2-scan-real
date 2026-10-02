#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pointlio_frame_adapter: 把 NX 本地 Point-LIO 的 /Odometry(camera_init/body) 适配为 map/base_footprint。
  1) 广播 TF map -> base_footprint（直接取位姿值，不做额外旋转）
  2) 发布 /quad_0/lidar_pose（frame=map, child=base_footprint, 位姿数值不变）
  只改 frame 名、不改姿态数值；base_footprint->base 静态外参仍由 real.launch 提供。"""
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

rospy.init_node("pointlio_frame_adapter", anonymous=True)
pub_lidar = rospy.Publisher("/quad_0/lidar_pose", Odometry, queue_size=10)
br = tf2_ros.TransformBroadcaster()

def cb(msg):
    p = msg.pose.pose
    ts = TransformStamped()
    ts.header.stamp = msg.header.stamp
    ts.header.frame_id = "map"
    ts.child_frame_id = "base_footprint"
    ts.transform.translation.x = p.position.x
    ts.transform.translation.y = p.position.y
    ts.transform.translation.z = p.position.z
    ts.transform.rotation.x = p.orientation.x
    ts.transform.rotation.y = p.orientation.y
    ts.transform.rotation.z = p.orientation.z
    ts.transform.rotation.w = p.orientation.w
    br.sendTransform(ts)
    out = Odometry()
    out.header = msg.header
    out.header.frame_id = "map"
    out.child_frame_id = "base_footprint"
    out.pose.pose.position = p.position
    out.pose.pose.orientation = p.orientation
    out.pose.covariance = msg.pose.covariance
    out.twist = msg.twist
    pub_lidar.publish(out)

rospy.Subscriber("/Odometry", Odometry, cb, queue_size=1)
rospy.logwarn("pointlio_frame_adapter: broadcasting map->base_footprint + /quad_0/lidar_pose(frame=map)")
rospy.spin()
