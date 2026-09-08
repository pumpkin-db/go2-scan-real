#!/usr/bin/env python3
"""Expose NX FAST-LIO through the established go2-scan ROS interface.

FAST-LIO publishes an end-of-scan IMU pose and registered world cloud with the
same timestamp.  This adapter pairs those messages exactly, converts the cloud
back to the calibrated LiDAR frame for SCAN, and retains the historical topic
names used by launch_real_rviz.sh and the rest of go2-scan.
"""
import copy
import math

import message_filters
import numpy as np
import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf.transformations import quaternion_from_euler, quaternion_from_matrix, quaternion_matrix


BODY_TRANSLATION = np.array([0.10700, -0.02329, -0.21697], dtype=float)
BODY_QUATERNION = quaternion_from_euler(0.0, math.radians(-8.0), math.pi, 'sxyz')


def rigid_from_pose(pose):
    q = pose.orientation
    quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
    if not np.all(np.isfinite(quat)) or np.linalg.norm(quat) < 1e-6:
        raise ValueError('invalid odometry quaternion')
    transform = quaternion_matrix(quat)
    transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    if not np.all(np.isfinite(transform)):
        raise ValueError('invalid odometry position')
    return transform


def odometry_from_transform(source, transform, child_frame):
    output = copy.deepcopy(source)
    output.header.frame_id = 'map'
    output.child_frame_id = child_frame
    output.pose.pose.position.x = transform[0, 3]
    output.pose.pose.position.y = transform[1, 3]
    output.pose.pose.position.z = transform[2, 3]
    q = quaternion_from_matrix(transform)
    output.pose.pose.orientation.x = q[0]
    output.pose.pose.orientation.y = q[1]
    output.pose.pose.orientation.z = q[2]
    output.pose.pose.orientation.w = q[3]
    return output


def world_to_lidar(message, world_lidar):
    fields = {field.name: field for field in message.fields}
    if any(name not in fields or fields[name].datatype != 7 or fields[name].count != 1
           for name in ('x', 'y', 'z')):
        raise ValueError('FAST-LIO cloud must contain float32 x/y/z')
    raw = bytearray(message.data)
    endian = '>' if message.is_bigendian else '<'
    dtype = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': [endian + 'f4'] * 3,
        'offsets': [fields[name].offset for name in ('x', 'y', 'z')],
        'itemsize': message.point_step,
    })
    points = np.ndarray((message.height, message.width), dtype=dtype, buffer=raw,
                        strides=(message.row_step, message.point_step))
    xyz = np.stack([points[name] for name in ('x', 'y', 'z')], axis=-1)
    local = (xyz - world_lidar[:3, 3]) @ world_lidar[:3, :3]
    for column, name in enumerate(('x', 'y', 'z')):
        points[name] = local[..., column]
    output = copy.copy(message)
    output.header = copy.deepcopy(message.header)
    output.header.frame_id = 'livox_frame'
    output.data = bytes(raw)
    return output


class FastlioOutputAdapter:
    def __init__(self):
        config_path = rospy.get_param('~config')
        with open(config_path, encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        mapping = config['mapping']
        if mapping.get('extrinsic_est_en', False):
            raise ValueError('navigation adapter requires fixed FAST-LIO extrinsics')
        self.lidar_rotation = np.asarray(mapping['extrinsic_R'], dtype=float).reshape(3, 3)
        self.lidar_translation = np.asarray(mapping['extrinsic_T'], dtype=float)
        if not np.all(np.isfinite(self.lidar_rotation)) or not np.all(np.isfinite(self.lidar_translation)):
            raise ValueError('invalid FAST-LIO extrinsics')
        self.imu_lidar = np.eye(4)
        self.imu_lidar[:3, :3] = self.lidar_rotation
        self.imu_lidar[:3, 3] = self.lidar_translation
        self.imu_body = quaternion_matrix(BODY_QUATERNION)
        self.imu_body[:3, 3] = BODY_TRANSLATION

        self.imu_pub = rospy.Publisher('/LIO/odom_imu', Odometry, queue_size=20)
        self.vehicle_pub = rospy.Publisher('/LIO/odom_vehicle', Odometry, queue_size=20)
        self.body_pub = rospy.Publisher('/quad_0/body_pose', Odometry, queue_size=20)
        self.lidar_pose_pub = rospy.Publisher('/quad_0/lidar_pose', Odometry, queue_size=20)
        self.compat_odom_pub = rospy.Publisher('/pointlio/Odometry', Odometry, queue_size=20)
        self.local_cloud_pub = rospy.Publisher('/LIO/clouds_lidar', PointCloud2, queue_size=2)
        self.world_cloud_pub = rospy.Publisher('/mid360_points', PointCloud2, queue_size=2)
        self.registered_pub = rospy.Publisher('/registered_scan', PointCloud2, queue_size=2)
        self.compat_cloud_pub = rospy.Publisher('/pointlio/cloud_registered', PointCloud2, queue_size=2)

        lidar_static = TransformStamped()
        lidar_static.header.stamp = rospy.Time.now()
        lidar_static.header.frame_id = 'base_footprint'
        lidar_static.child_frame_id = 'livox_frame'
        lidar_static.transform.translation.x = self.lidar_translation[0]
        lidar_static.transform.translation.y = self.lidar_translation[1]
        lidar_static.transform.translation.z = self.lidar_translation[2]
        q = quaternion_from_matrix(self.imu_lidar)
        lidar_static.transform.rotation.x = q[0]
        lidar_static.transform.rotation.y = q[1]
        lidar_static.transform.rotation.z = q[2]
        lidar_static.transform.rotation.w = q[3]

        # Keep the established Go2 URDF root aligned with the calibrated body
        # centre.  This is T_imu_body, separate from FAST-LIO's T_imu_lidar.
        body_static = TransformStamped()
        body_static.header.stamp = lidar_static.header.stamp
        body_static.header.frame_id = 'base_footprint'
        body_static.child_frame_id = 'base'
        body_static.transform.translation.x = BODY_TRANSLATION[0]
        body_static.transform.translation.y = BODY_TRANSLATION[1]
        body_static.transform.translation.z = BODY_TRANSLATION[2]
        q = quaternion_from_matrix(self.imu_body)
        body_static.transform.rotation.x = q[0]
        body_static.transform.rotation.y = q[1]
        body_static.transform.rotation.z = q[2]
        body_static.transform.rotation.w = q[3]
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_broadcaster.sendTransform([lidar_static, body_static])
        self.dynamic_broadcaster = tf2_ros.TransformBroadcaster()

        odom_sub = message_filters.Subscriber('/Odometry', Odometry, queue_size=30)
        cloud_sub = message_filters.Subscriber('/cloud_registered', PointCloud2, queue_size=6)
        odom_sub.registerCallback(self.on_odometry)
        self.sync = message_filters.TimeSynchronizer([cloud_sub, odom_sub], 30)
        self.sync.registerCallback(self.on_pair)
        self.odom_sub = odom_sub
        self.cloud_sub = cloud_sub
        self.pairs = 0
        rospy.loginfo('FAST-LIO adapter keeps legacy go2-scan topics; exact-time cloud/odom pairing enabled')

    @staticmethod
    def normalize_odom(message):
        output = copy.deepcopy(message)
        output.header.frame_id = 'map'
        output.child_frame_id = 'base_footprint'
        return output

    def on_odometry(self, message):
        try:
            normalized = self.normalize_odom(message)
            world_imu = rigid_from_pose(normalized.pose.pose)
            lidar = odometry_from_transform(normalized, world_imu @ self.imu_lidar, 'livox_frame')
            body = odometry_from_transform(normalized, world_imu @ self.imu_body, 'go2_body')
        except ValueError as error:
            rospy.logerr_throttle(2.0, 'Rejected FAST-LIO odometry: %s', error)
            return
        root_tf = TransformStamped()
        root_tf.header = copy.deepcopy(normalized.header)
        root_tf.child_frame_id = normalized.child_frame_id
        root_tf.transform.translation.x = normalized.pose.pose.position.x
        root_tf.transform.translation.y = normalized.pose.pose.position.y
        root_tf.transform.translation.z = normalized.pose.pose.position.z
        root_tf.transform.rotation = normalized.pose.pose.orientation
        self.dynamic_broadcaster.sendTransform(root_tf)
        self.imu_pub.publish(normalized)
        self.compat_odom_pub.publish(normalized)
        self.lidar_pose_pub.publish(lidar)
        self.body_pub.publish(body)
        self.vehicle_pub.publish(body)

    def on_pair(self, cloud, odometry):
        try:
            normalized = self.normalize_odom(odometry)
            world_imu = rigid_from_pose(normalized.pose.pose)
            local = world_to_lidar(cloud, world_imu @ self.imu_lidar)
        except (TypeError, ValueError) as error:
            rospy.logerr_throttle(2.0, 'Rejected FAST-LIO cloud pair: %s', error)
            return
        world = copy.copy(cloud)
        world.header = copy.deepcopy(cloud.header)
        world.header.frame_id = 'map'
        self.local_cloud_pub.publish(local)
        self.world_cloud_pub.publish(world)
        self.registered_pub.publish(world)
        self.compat_cloud_pub.publish(world)
        self.pairs += 1
        rospy.loginfo_throttle(10.0, 'FAST-LIO exact-time adapted pairs: %d', self.pairs)


if __name__ == '__main__':
    rospy.init_node('fastlio_output_adapter')
    FastlioOutputAdapter()
    rospy.spin()
