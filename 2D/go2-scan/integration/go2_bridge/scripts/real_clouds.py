#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
go2_bridge: 高分辨率强度点云 /real_clouds（仅 RViz 显示，不进算法链）。

输入  /livox/lidar = livox_ros_driver2/CustomMsg（raw 点云，含 x/y/z/reflectivity/tag/line）
输出  /real_clouds  (sensor_msgs/PointCloud2)
  x y z + intensity(=reflectivity, float32)，frame=livox_frame，voxel 0.02m 下采样。

不做逐点坐标变换——跨帧显示靠 TF base_footprint(IMU)->livox_frame 解析。
点数可减少，但空间分辨率必须明显高于 /scan_map（0.05m 体素累积）。
"""
import sys
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

# livox_ros_driver2 位于 catkin_ws，不在 real.launch 的 PYTHONPATH 里，须显式补路径
sys.path.insert(0, '/home/unitree/catkin_ws/devel/lib/python3/dist-packages')
from livox_ros_driver2.msg import CustomMsg


def main():
    rospy.init_node('real_clouds')
    voxel = rospy.get_param('~voxel_size', 0.02)
    max_pts = rospy.get_param('~max_points', 200000)
    out_frame = rospy.get_param('~out_frame', 'livox_frame')
    pub = rospy.Publisher('/real_clouds', PointCloud2, queue_size=1)

    def cb(msg):
        n = msg.point_num
        if n == 0:
            return
        # CustomMsg.points -> (N,4): x y z reflectivity
        pts = np.array([[p.x, p.y, p.z, float(p.reflectivity)] for p in msg.points],
                       dtype=np.float32)
        if pts.shape[0] == 0:
            return
        xyz = pts[:, :3].astype(np.float64)
        intensity = pts[:, 3].astype(np.float32)

        # 轻量 voxel 下采样：保留每体素首个点（含其强度）
        keys = np.floor(xyz / voxel).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        xyz = xyz[idx]
        intensity = intensity[idx]
        if len(xyz) > max_pts:
            sel = np.random.choice(len(xyz), max_pts, replace=False)
            xyz = xyz[sel]
            intensity = intensity[sel]

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
        ]
        buf = np.zeros(len(xyz), dtype=[('x', '<f4'), ('y', '<f4'),
                                        ('z', '<f4'), ('intensity', '<f4')])
        buf['x'] = xyz[:, 0]
        buf['y'] = xyz[:, 1]
        buf['z'] = xyz[:, 2]
        buf['intensity'] = intensity
        out = PointCloud2()
        out.header = msg.header
        out.header.frame_id = msg.header.frame_id or out_frame
        out.height = 1
        out.width = len(xyz)
        out.fields = fields
        out.is_bigendian = False
        out.point_step = 16
        out.row_step = 16 * len(xyz)
        out.data = buf.tobytes()
        pub.publish(out)

    rospy.Subscriber('/livox/lidar', CustomMsg, cb, queue_size=1)
    rospy.loginfo('[real_clouds] /livox/lidar(CustomMsg) -> /real_clouds voxel=%.2fm frame=%s',
                  voxel, out_frame)
    rospy.spin()


if __name__ == '__main__':
    main()
