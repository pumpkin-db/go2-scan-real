cmake_minimum_required(VERSION 3.10)
project(cmd_vel_bridge CXX)

# ---- C++17 (required by unitree_sdk2) ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Make sure ROS1 Noetic is on the CMake search path even if the user
# forgot to source /opt/ros/noetic/setup.bash.
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/noetic")

# ---- ROS1 dependencies (roscpp + geometry_msgs/Twist) via catkin finder ----
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)

# ---- unitree_sdk2 + its bundled CycloneDDS (ddsc / ddscxx) ----
# The release tree vendors the ARM64 SDK under ~/Go2/third_party.  Keep a
# cache/env override for unusual installations without baking one NX user's
# home directory into the build.
get_filename_component(GO2_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
set(UNITREE_SDK_DEFAULT "${GO2_ROOT}/third_party/unitree_sdk2")
if(DEFINED ENV{UNITREE_SDK_ROOT} AND NOT "$ENV{UNITREE_SDK_ROOT}" STREQUAL "")
    set(UNITREE_SDK_DEFAULT "$ENV{UNITREE_SDK_ROOT}")
endif()
set(UNITREE_SDK_ROOT "${UNITREE_SDK_DEFAULT}" CACHE PATH "Path to unitree_sdk2")
set(UNITREE_SDK_INC    "${UNITREE_SDK_ROOT}/include")
set(UNITREE_SDK_LIBDIR "${UNITREE_SDK_ROOT}/lib/aarch64")
set(UNITREE_TP_INC     "${UNITREE_SDK_ROOT}/thirdparty/include" "${UNITREE_SDK_ROOT}/thirdparty/include/ddscxx")
set(UNITREE_TP_LIBDIR  "${UNITREE_SDK_ROOT}/thirdparty/lib/aarch64")

# Use deterministic paths so an old CMake find_library cache cannot silently
# retain libraries from outside this release tree.
set(UNITREE_SDK_LIB "${UNITREE_SDK_LIBDIR}/libunitree_sdk2.a")
set(DDSC_LIB "${UNITREE_TP_LIBDIR}/libddsc.so")
set(DDSCXX_LIB "${UNITREE_TP_LIBDIR}/libddscxx.so")

if (NOT UNITREE_SDK_LIB OR NOT DDSC_LIB OR NOT DDSCXX_LIB)
    message(FATAL_ERROR
        "Missing one of: unitree_sdk2 / ddsc / ddscxx under ${UNITREE_SDK_ROOT}\n"
        "  unitree_sdk2 = ${UNITREE_SDK_LIB}\n"
        "  ddsc         = ${DDSC_LIB}\n"
        "  ddscxx       = ${DDSCXX_LIB}\n")
endif ()

message(STATUS "unitree_sdk2 : ${UNITREE_SDK_LIB}")
message(STATUS "ddsc         : ${DDSC_LIB}")
message(STATUS "ddscxx       : ${DDSCXX_LIB}")

find_package(Threads REQUIRED)

# ---- The executable ----
add_executable(cmd_vel_bridge cmd_vel_bridge.cpp)

target_include_directories(cmd_vel_bridge PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${UNITREE_SDK_INC}
    ${UNITREE_TP_INC}
)

# Link order mirrors unitree_sdk2's own CMakeLists:
#   unitree_sdk2 INTERFACE ddsc ddscxx Threads::Threads
target_link_libraries(cmd_vel_bridge
    ${catkin_LIBRARIES}
    ${UNITREE_SDK_LIB}
    ${DDSCXX_LIB}
    ${DDSC_LIB}
    Threads::Threads
)

# ---- go2_ensure_stand (确认/完成站立) ----
add_executable(go2_ensure_stand go2_ensure_stand.cpp)
target_include_directories(go2_ensure_stand PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${UNITREE_SDK_INC}
    ${UNITREE_TP_INC}
)
target_link_libraries(go2_ensure_stand
    ${catkin_LIBRARIES}
    ${UNITREE_SDK_LIB}
    ${DDSCXX_LIB}
    ${DDSC_LIB}
    Threads::Threads
)
# ---- go2_state_probe (诊断: 实时打印 sportmodestate mode) ----
add_executable(go2_state_probe go2_state_probe.cpp)
target_include_directories(go2_state_probe PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${UNITREE_SDK_INC}
    ${UNITREE_TP_INC}
)
target_link_libraries(go2_state_probe
    ${catkin_LIBRARIES}
    ${UNITREE_SDK_LIB}
    ${DDSCXX_LIB}
    ${DDSC_LIB}
    Threads::Threads
)
# ---- go2_getstate_probe (GetState api 1034 查询) ----
add_executable(go2_getstate_probe go2_getstate_probe.cpp)
target_include_directories(go2_getstate_probe PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${UNITREE_SDK_INC}
    ${UNITREE_TP_INC}
)
target_link_libraries(go2_getstate_probe
    ${catkin_LIBRARIES}
    ${UNITREE_SDK_LIB}
    ${DDSCXX_LIB}
    ${DDSC_LIB}
    Threads::Threads
)
# ---- go2_rec_stand_once (安全实验: 站立时调一次 RecoveryStand) ----
add_executable(go2_rec_stand_once go2_rec_stand_once.cpp)
target_include_directories(go2_rec_stand_once PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${UNITREE_SDK_INC}
    ${UNITREE_TP_INC}
)
target_link_libraries(go2_rec_stand_once
    ${catkin_LIBRARIES}
    ${UNITREE_SDK_LIB}
    ${DDSCXX_LIB}
    ${DDSC_LIB}
    Threads::Threads
)
# ---- go2_recovery_stand (正式: 一次性 RecoveryStand + 等5s) ----
add_executable(go2_recovery_stand go2_recovery_stand.cpp)
target_include_directories(go2_recovery_stand PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${UNITREE_SDK_INC}
    ${UNITREE_TP_INC}
)
target_link_libraries(go2_recovery_stand
    ${catkin_LIBRARIES}
    ${UNITREE_SDK_LIB}
    ${DDSCXX_LIB}
    ${DDSC_LIB}
    Threads::Threads
)

# StandUp entry for the independent NX launcher.
add_executable(go2_standup go2_standup.cpp)
target_include_directories(go2_standup PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${UNITREE_SDK_INC}
    ${UNITREE_TP_INC}
)
target_link_libraries(go2_standup
    ${catkin_LIBRARIES}
    ${UNITREE_SDK_LIB}
    ${DDSCXX_LIB}
    ${DDSC_LIB}
    Threads::Threads
)
