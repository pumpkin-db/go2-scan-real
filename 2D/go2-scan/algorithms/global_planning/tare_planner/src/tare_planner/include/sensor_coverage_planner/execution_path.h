#pragma once

#include <Eigen/Core>
#include <nav_msgs/Path.h>
#include <vector>

namespace sensor_coverage_planner_3d_ns
{
// Export only the already selected ordered branch; never infer a shortcut
// between the robot and a geometrically nearby point on an exploration loop.
inline nav_msgs::Path MakeExecutionPath(const std::vector<Eigen::Vector3d>& nodes,
                                       int start, int end, const Eigen::Vector3d& robot,
                                       const Eigen::Vector3d& target)
{
  nav_msgs::Path path;
  path.header.frame_id = "map";
  if (start < 0 || end < 0 || start >= static_cast<int>(nodes.size()) ||
      end >= static_cast<int>(nodes.size()) || !robot.allFinite() || !target.allFinite())
    return path;
  const auto append = [&path](const Eigen::Vector3d& point) {
    if (!point.allFinite()) return false;
    if (!path.poses.empty())
    {
      const auto& last = path.poses.back().pose.position;
      if ((point - Eigen::Vector3d(last.x, last.y, last.z)).norm() < 1e-6) return true;
    }
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    pose.pose.position.z = point.z();
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
    return true;
  };
  append(robot);
  const int step = end >= start ? 1 : -1;
  for (int i = start; i != end; i += step)
    if (!append(nodes[i])) { path.poses.clear(); return path; }
  // The chosen lookahead may lie partway along the final edge.
  if (!append(target)) path.poses.clear();
  return path;
}
}  // namespace sensor_coverage_planner_3d_ns
