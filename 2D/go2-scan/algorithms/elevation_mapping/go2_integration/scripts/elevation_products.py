#!/usr/bin/env python3
"""Bounded display map and persistent 2-D exploration map from local elevation products.

The elevation estimate and its terrain layers come from ANYbotics elevation_mapping
and grid_map filters.  This node only performs message-level accumulation:
  * local binary traversability -> persistent /projected_map for ARiADNE;
  * local elevation point cloud -> bounded /global_elevation_map for RViz;
  * nearby ground samples -> /floor_context/z_ref for visualization and target Z.
"""

import math
import time
from collections import OrderedDict

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float64, Header


class ElevationProducts:
    def __init__(self):
        self.map_resolution = float(rospy.get_param('~map_resolution', 0.1))
        self.global_resolution = float(rospy.get_param('~global_resolution', 0.2))
        self.global_max_cells = int(rospy.get_param('~global_max_cells', 100000))
        self.global_publish_hz = float(rospy.get_param('~global_publish_hz', 0.5))
        self.floor_radius = float(rospy.get_param('~floor_radius', 0.75))
        self.floor_min_points = int(rospy.get_param('~floor_min_points', 3))
        self.occupied_threshold = int(rospy.get_param('~occupied_threshold', 50))
        if self.map_resolution <= 0 or self.global_resolution <= 0:
            raise ValueError('map resolutions must be positive')
        if self.global_max_cells < 1 or self.global_publish_hz <= 0:
            raise ValueError('global map limits must be positive')

        self.robot_xy = None
        self.cells_2d = {}
        self.global_cells = OrderedDict()
        self.last_cloud = None
        self.projected_pub = rospy.Publisher('/projected_map', OccupancyGrid,
                                             queue_size=1, latch=True)
        self.global_pub = rospy.Publisher('/global_elevation_map', PointCloud2,
                                          queue_size=1, latch=True)
        self.floor_pub = rospy.Publisher('/floor_context/z_ref', Float64,
                                         queue_size=1, latch=True)
        rospy.Subscriber('/local_traversability_map', OccupancyGrid,
                         self.map_callback, queue_size=1)
        rospy.Subscriber('/local_elevation_cloud', PointCloud2,
                         self.cloud_callback, queue_size=1)
        rospy.Subscriber('/LIO/odom_vehicle', Odometry,
                         self.odom_callback, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / self.global_publish_hz), self.publish_global)
        rospy.loginfo('[go2_elevation] products: local %.2fm, global %.2fm, cap=%d',
                      self.map_resolution, self.global_resolution,
                      self.global_max_cells)

    def odom_callback(self, message):
        p = message.pose.pose.position
        if math.isfinite(p.x) and math.isfinite(p.y):
            self.robot_xy = (p.x, p.y)

    def map_callback(self, message):
        if message.info.resolution <= 0 or not message.data:
            return
        source_resolution = message.info.resolution
        origin_x = message.info.origin.position.x
        origin_y = message.info.origin.position.y
        width = message.info.width
        for index, value in enumerate(message.data):
            if value < 0:
                continue
            sx = index % width
            sy = index // width
            x = origin_x + (sx + 0.5) * source_resolution
            y = origin_y + (sy + 0.5) * source_resolution
            key = (int(math.floor(x / self.map_resolution)),
                   int(math.floor(y / self.map_resolution)))
            self.cells_2d[key] = 100 if value >= self.occupied_threshold else 0
        self.publish_projected(message.header)

    def publish_projected(self, header):
        if not self.cells_2d:
            return
        xs = [key[0] for key in self.cells_2d]
        ys = [key[1] for key in self.cells_2d]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        width = max_x - min_x + 1
        height = max_y - min_y + 1
        data = [-1] * (width * height)
        for (ix, iy), value in self.cells_2d.items():
            data[(iy - min_y) * width + ix - min_x] = value
        output = OccupancyGrid()
        output.header = header
        output.header.frame_id = 'map'
        output.info.map_load_time = header.stamp
        output.info.resolution = self.map_resolution
        output.info.width = width
        output.info.height = height
        output.info.origin.position.x = min_x * self.map_resolution
        output.info.origin.position.y = min_y * self.map_resolution
        output.info.origin.orientation.w = 1.0
        output.data = data
        self.projected_pub.publish(output)

    def cloud_callback(self, message):
        fields = {field.name for field in message.fields}
        if not {'x', 'y', 'z'}.issubset(fields):
            rospy.logerr_throttle(2.0, '[go2_elevation] local cloud lacks xyz fields')
            return
        points = np.asarray(list(point_cloud2.read_points(
            message, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=np.float64)
        if points.size == 0:
            return
        points = points.reshape((-1, 3))
        self.last_cloud = points
        self.update_floor(points)
        buckets = {}
        for x, y, z in points:
            key = (int(math.floor(x / self.global_resolution)),
                   int(math.floor(y / self.global_resolution)))
            buckets.setdefault(key, []).append(float(z))
        now = time.monotonic()
        for key, heights in buckets.items():
            z = float(np.median(heights))
            previous = self.global_cells.pop(key, None)
            if previous is None:
                value = (z, 1, now)
            else:
                old_z, count, _ = previous
                count = min(count + 1, 20)
                value = (old_z + (z - old_z) / count, count, now)
            self.global_cells[key] = value
        while len(self.global_cells) > self.global_max_cells:
            self.global_cells.popitem(last=False)

    def update_floor(self, points):
        if self.robot_xy is None:
            return
        dx = points[:, 0] - self.robot_xy[0]
        dy = points[:, 1] - self.robot_xy[1]
        nearby = points[dx * dx + dy * dy <= self.floor_radius * self.floor_radius, 2]
        if nearby.size >= self.floor_min_points:
            self.floor_pub.publish(Float64(float(np.median(nearby))))

    def publish_global(self, _event):
        if not self.global_cells:
            return
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('elevation', 12, PointField.FLOAT32, 1),
        ]
        points = []
        half = 0.5 * self.global_resolution
        for (ix, iy), (z, _count, _updated) in self.global_cells.items():
            points.append((ix * self.global_resolution + half,
                           iy * self.global_resolution + half, z, z))
        header = Header(stamp=rospy.Time.now(), frame_id='map')
        self.global_pub.publish(point_cloud2.create_cloud(header, fields, points))


if __name__ == '__main__':
    rospy.init_node('go2_elevation_products')
    ElevationProducts()
    rospy.spin()
