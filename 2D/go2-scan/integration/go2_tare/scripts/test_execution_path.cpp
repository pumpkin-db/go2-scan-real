#include <sensor_coverage_planner/execution_path.h>
#include <cassert>
#include <iostream>
#include <limits>

using Eigen::Vector3d;
using sensor_coverage_planner_3d_ns::MakeExecutionPath;

int main()
{
  std::vector<Vector3d> nodes{{0,0,0}, {2,0,0}, {2,3,0}, {0,3,0}, {0,0,0}};
  auto path = MakeExecutionPath(nodes, 0, 2, nodes[0], Vector3d(2,1,0));
  assert(path.poses.size() == 3);
  assert(path.poses[1].pose.position.x == 2 && path.poses[1].pose.position.y == 0);
  assert(path.poses.back().pose.position.y == 1); // preserve elbow, clip final edge
  path = MakeExecutionPath(nodes, 4, 2, nodes[4], Vector3d(1,3,0));
  assert(path.poses.size() == 3);
  assert(path.poses[1].pose.position.x == 0 && path.poses[1].pose.position.y == 3);
  assert(path.poses.back().pose.position.x == 1); // reverse branch of the loop
  path = MakeExecutionPath(nodes, 1, 2, Vector3d(1.9,0,0), nodes[2]);
  assert(path.poses.front().pose.position.x == 1.9);
  assert(path.poses[1].pose.position.x == 2);
  assert(path.poses.back().pose.position.y == 3);
  assert(MakeExecutionPath(nodes, -1, 2, nodes[0], nodes[2]).poses.empty());
  assert(MakeExecutionPath({}, 0, 0, nodes[0], nodes[2]).poses.empty());
  assert(MakeExecutionPath(nodes, 0, 2, nodes[0],
         Vector3d(std::numeric_limits<double>::quiet_NaN(),0,0)).poses.empty());
  path = MakeExecutionPath(nodes, 0, 0, nodes[0], nodes[0]);
  assert(path.poses.size() == 1); // no artificial nonzero task
  std::cout << "Execution path order/corner/partial-edge/validation: PASS\n";
}
