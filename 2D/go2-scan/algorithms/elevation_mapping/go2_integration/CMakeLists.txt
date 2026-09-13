cmake_minimum_required(VERSION 3.0.2)
project(go2_elevation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  grid_map_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/fastlio_pose_covariance.py
  scripts/elevation_products.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
