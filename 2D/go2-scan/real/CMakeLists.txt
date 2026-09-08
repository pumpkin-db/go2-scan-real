cmake_minimum_required(VERSION 3.0.2)
project(go2_mid360_raw)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS mid360_raw_accumulator.py mid360_view_bridge.py mid360_remote_topics.py
  fastlio_output_adapter.py cmd_vel_safety_gate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
