#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
glue 累积节点：把每帧 /mid360_points（已是世界系，frame=world，由 livox 插件直接输出）
            按距离过滤后体素累积，发布 /scan_map。

按固定时间窗口分段累计：每 20 秒整批清空一次，然后重新累计。

输入：/mid360_points（世界系点云，frame=world）
输出：/scan_map（累积体素点云，frame world，1Hz）
服务：/scan_map/save 保存、/scan_map/clear 清空
"""
import rospy
import numpy as np
import threading
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse


class ScanAccumulator:
    def __init__(self):
        self.pose = None  # 仅作距离过滤参考基准（不做坐标变换）
        self.voxel = rospy.get_param('~voxel_size', 0.20)
        self.reset_interval = rospy.get_param('~reset_interval', 20.0)
        self.max_range = rospy.get_param('~max_range', 12.0)
        self.min_range = rospy.get_param('~min_range', 0.3)
        self.publish_max_points = rospy.get_param('~publish_max_points', 100000)
        self.save_dir = rospy.get_param('~save_dir', '/tmp')
        # (ix,iy,iz) -> [intensity_sum, hit_count]。几何仍为纯累积，
        # 强度取该体素历次回波均值，仅用于 /scan_map 的 RViz 着色。
        self.cells = {}
        self.cells_lock = threading.Lock()

        self.pub = rospy.Publisher('/scan_map', PointCloud2, queue_size=1)
        self.sub_cloud = rospy.Subscriber('/mid360_points', PointCloud2, self.cloud_cb, queue_size=1)
        self.sub_pose = rospy.Subscriber('/quad_0/lidar_pose', Odometry, self.pose_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0), self.publish_cb)
        self.reset_timer = rospy.Timer(rospy.Duration(self.reset_interval), self.reset_cb)
        rospy.Service('/scan_map/save', Trigger, self.save_cb)
        rospy.Service('/scan_map/clear', Empty, self.clear_cb)
        rospy.loginfo('[scan_cloud_accumulator] ready: world-frame segmented map, voxel=%.2fm, reset=%.1fs',
                      self.voxel, self.reset_interval)

    def pose_cb(self, msg):
        # 仅保存用作距离过滤基准（排除狗自身点），不用于坐标变换
        self.pose = msg.pose.pose

    def cloud_cb(self, msg):
        # 2026-08-26 改 numpy 直析：noetic pc2.read_points 对本链路点云存在
        # unpack_from 越界(struct.error)，逐帧异常导致积累量骤减（bench_fix2
        # 实证 scan 仅 4.9k 点）。三字段 step16 布局由 velodyne 插件保证。
        n = msg.width * msg.height
        if n == 0 or len(msg.data) < n * msg.point_step:
            return
        raw = bytes(msg.data)
        arr = np.frombuffer(raw, dtype=np.uint8).reshape(n, msg.point_step)
        xyz = arr[:, :12].copy().view(np.float32).reshape(n, 3)
        valid_xyz = np.isfinite(xyz).all(axis=1)
        pts = xyz[valid_xyz].astype(np.float64)
        if len(pts) == 0:
            return
        intensity = np.zeros(n, dtype=np.float32)
        intensity_field = next((field for field in msg.fields if field.name == 'intensity'), None)
        if (intensity_field is not None and intensity_field.datatype == PointField.FLOAT32 and
                intensity_field.count == 1 and intensity_field.offset + 4 <= msg.point_step):
            byte_order = '>' if msg.is_bigendian else '<'
            intensity = np.ndarray((n,), dtype=byte_order + 'f4', buffer=raw,
                                   offset=intensity_field.offset,
                                   strides=(msg.point_step,)).copy()
            intensity[~np.isfinite(intensity)] = 0.0
        intensity = intensity[valid_xyz]
        # 点云已是世界系（livox 插件直接输出），无需坐标变换
        # 距离过滤基准：用 dog 传感器位置（若无则原点）
        if self.pose is not None:
            origin = np.array([self.pose.position.x, self.pose.position.y, self.pose.position.z])
        else:
            origin = np.array([0.0, 0.0, 0.0])

        dist = np.linalg.norm(pts - origin, axis=1)
        mask = (dist >= self.min_range) & (dist <= self.max_range)
        pts_w = pts[mask]
        intensity_w = intensity[mask]
        if len(pts_w) == 0:
            return

        # 体素累积（floor 正确处理负数坐标，add-only 只加不删）
        keys = np.floor(pts_w / self.voxel).astype(np.int64)
        with self.cells_lock:
            for k, reflectivity in zip(keys, intensity_w):
                key = (int(k[0]), int(k[1]), int(k[2]))
                value = self.cells.get(key)
                if value is None:
                    self.cells[key] = [float(reflectivity), 1]
                else:
                    value[0] += float(reflectivity)
                    value[1] += 1

    def publish_cb(self, _event):
        with self.cells_lock:
            if not self.cells:
                return
            items = list(self.cells.items())
        # 锁内只取快照，后续转换/发布不阻塞点云回调。
        keys = np.array([item[0] for item in items], dtype=np.int64)
        intensities = np.array([item[1][0] / item[1][1] for item in items], dtype=np.float32)
        if len(keys) > self.publish_max_points:
            sel = np.random.choice(len(keys), self.publish_max_points, replace=False)
            keys = keys[sel]
            intensities = intensities[sel]
        pts = ((keys + 0.5) * self.voxel).astype(np.float32)
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'world'
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
        ]
        xyzi = np.ascontiguousarray(np.column_stack((pts, intensities)), dtype=np.float32)
        output = PointCloud2()
        output.header = header
        output.height = 1
        output.width = len(xyzi)
        output.fields = fields
        output.is_bigendian = False
        output.point_step = 16
        output.row_step = output.point_step * output.width
        output.is_dense = True
        output.data = xyzi.tobytes()
        self.pub.publish(output)

    def save_cb(self, _req):
        try:
            with self.cells_lock:
                items = list(self.cells.items())
            keys = np.array([item[0] for item in items], dtype=np.int64)
            intensities = np.array([item[1][0] / item[1][1] for item in items], dtype=np.float32)
            pts = ((keys + 0.5) * self.voxel).astype(np.float32)
            pcd_path = self.save_dir + '/scan_map_occupancy.pcd'
            with open(pcd_path, 'w') as f:
                f.write('# .PCD v0.7 - Point Cloud Data file format\n')
                f.write('VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n')
                f.write('TYPE F F F F\nCOUNT 1 1 1 1\n')
                f.write('WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n' % len(pts))
                f.write('POINTS %d\nDATA ascii\n' % len(pts))
                for row, reflectivity in zip(pts, intensities):
                    f.write('%.4f %.4f %.4f %.3f\n' %
                            (row[0], row[1], row[2], reflectivity))
            return TriggerResponse(success=True, message='saved %d points to %s' % (len(pts), pcd_path))
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

    def clear_cb(self, _req):
        with self.cells_lock:
            self.cells.clear()
        return EmptyResponse()

    def reset_cb(self, _event):
        with self.cells_lock:
            removed = len(self.cells)
            self.cells.clear()
        rospy.loginfo('[scan_cloud_accumulator] %.1fs window reset: cleared %d voxels',
                      self.reset_interval, removed)


if __name__ == '__main__':
    rospy.init_node('scan_cloud_accumulator')
    ScanAccumulator()
    rospy.spin()
