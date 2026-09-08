#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
点云近距过滤节点：剔除传感器原点附近的无效点，发布 /mid360_points_clean。

为什么需要（2026-08-22 实测，见 third_party.md）：
1. livox 插件把未击中射线的 range 置 0 后仍按 range*axis 发布 → 这些点精确落在
   传感器原点；
2. MID360 装在 Go2 顶部，正常扫描也会持续打到自己身体。
这两类点进入下游会把机器人周围格子染成占据 → ARiADNE 与 SCAN 对同一格子判断
相反 → 目标被拒 → 狗站桩死锁。

做法：订阅世界系 /mid360_points + /quad_0/lidar_pose（传感器世界位姿），
剔除与传感器水平距离 < min_range 的点，按原始字段布局重打包发布
/mid360_points_clean（保留全部字段与 frame_id=world，下游免变换不变）。
仅在 global_planner!=tare 分支启动（TARE 路径保持基线行为）。

【教训留档 2026-08-24】曾在此加过「帧级健全性门控」（z_std 等判据丢弃异常帧），
已撤除：MID360 非重复扫描本来就会产出单仰角环帧（整帧 z 几乎同值），属正常
数据形态，误判成退化帧会丢掉大量合法观测。
"""
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


class CloudRangeFilter:
    def __init__(self):
        self.min_range = rospy.get_param('~min_range', 0.7)
        # 远距裁剪（2026-08-26 效用全零根因修复）：ray 物理量程 40m 下远点若放行，
        # octomap(max_range=6m) 只清空其沿途 free 而不产生占据 → 未探索区被虚报为
        # 自由空间、frontier 枯竭。裁到 planner 同款量程后「看不见=unknown」成立。
        # 0 = 禁用（保持旧行为）。
        self.max_range = rospy.get_param('~max_range', 0.0)
        # Optional AR-only temporal evidence gate.  A voxel is counted at most
        # once per input frame, so many returns in one scan cannot by themselves
        # turn a random speck into an obstacle.  Defaults preserve old behavior;
        # the real FAST-LIO launch explicitly requests two-frame confirmation.
        self.confirmation_frames = int(rospy.get_param('~confirmation_frames', 1))
        self.confirmation_voxel = float(rospy.get_param('~confirmation_voxel', 0.1))
        self.confirmation_window = float(rospy.get_param('~confirmation_window', 1.0))
        if self.confirmation_frames < 1:
            raise ValueError('confirmation_frames must be >= 1')
        if self.confirmation_voxel <= 0 or self.confirmation_window <= 0:
            raise ValueError('confirmation_voxel/window must be positive')
        self.frame_id = 0
        self.voxel_history = {}
        # 传感器位姿还没收到时丢弃点云帧（否则无法判距）
        self.sensor_pos = None
        self.pub = rospy.Publisher('/mid360_points_clean', PointCloud2, queue_size=1)
        self.sub_pose = rospy.Subscriber('/quad_0/lidar_pose', Odometry, self.pose_cb, queue_size=1)
        self.sub_cloud = rospy.Subscriber('/mid360_points', PointCloud2, self.cloud_cb, queue_size=1)
        rospy.loginfo('[cloud_range_filter] ready: /mid360_points -> /mid360_points_clean '
                      '(min=%.2fm max=%.2fm confirmation=%d frames, voxel=%.2fm, window=%.2fs)',
                      self.min_range, self.max_range, self.confirmation_frames,
                      self.confirmation_voxel, self.confirmation_window)

    def pose_cb(self, msg):
        p = msg.pose.pose.position
        self.sensor_pos = np.array([p.x, p.y, p.z])

    def cloud_cb(self, msg):
        if self.sensor_pos is None:
            return
        # 按消息自带字段布局解析（保留 x/y/z/intensity 等全部字段）
        dt = np.dtype({'names': [f.name for f in msg.fields],
                       'formats': [np.float32] * len(msg.fields),
                       'offsets': [f.offset for f in msg.fields],
                       'itemsize': msg.point_step})
        try:
            arr = np.frombuffer(bytes(msg.data), dtype=dt)
        except ValueError:
            rospy.logwarn_throttle(10, '[cloud_range_filter] 字段布局非全 float32，跳过该帧')
            return
        xy = np.stack([arr['x'], arr['y']], axis=-1).astype(np.float64)
        d = np.linalg.norm(xy - self.sensor_pos[:2], axis=1)
        keep = d >= self.min_range
        if self.max_range > 0:
            keep &= d <= self.max_range
        if self.confirmation_frames > 1 and np.any(keep):
            now = msg.header.stamp.to_sec()
            if not np.isfinite(now) or now <= 0:
                now = rospy.Time.now().to_sec()
            self.frame_id += 1
            candidate_indices = np.flatnonzero(keep)
            xyz = np.stack([arr['x'][candidate_indices], arr['y'][candidate_indices],
                            arr['z'][candidate_indices]], axis=-1).astype(np.float64)
            voxel_keys = np.floor(xyz / self.confirmation_voxel).astype(np.int64)
            unique_keys, inverse = np.unique(voxel_keys, axis=0, return_inverse=True)
            unique_confirmed = np.zeros(len(unique_keys), dtype=bool)
            for index, key_array in enumerate(unique_keys):
                key = tuple(int(value) for value in key_array)
                previous = self.voxel_history.get(key)
                if previous is None or now - previous[1] > self.confirmation_window:
                    count = 1
                elif previous[0] == self.frame_id:
                    count = previous[2]
                else:
                    count = min(self.confirmation_frames, previous[2] + 1)
                self.voxel_history[key] = (self.frame_id, now, count)
                unique_confirmed[index] = count >= self.confirmation_frames
            confirmed = unique_confirmed[inverse]
            keep[candidate_indices] &= confirmed
            if self.frame_id % 50 == 0:
                cutoff = now - self.confirmation_window
                self.voxel_history = {
                    key: value for key, value in self.voxel_history.items() if value[1] >= cutoff
                }
        kept = int(np.count_nonzero(keep))
        if kept == 0:
            return
        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = kept
        out.fields = msg.fields
        out.is_bigendian = False
        out.point_step = msg.point_step
        out.row_step = kept * msg.point_step
        out.data = np.ascontiguousarray(arr[keep]).tobytes()
        out.is_dense = True
        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('cloud_range_filter')
    CloudRangeFilter()
    rospy.spin()
