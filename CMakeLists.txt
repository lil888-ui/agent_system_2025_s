cmake_minimum_required(VERSION 3.5.0)
project(choreonoid_ros_tutorial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  geometry_msgs         # ← 追加
  tf2                   # ← 追加
  tf2_geometry_msgs     # ← 追加
  choreonoid
)

catkin_package(
  SKIP_CMAKE_CONFIG_GENERATION
  SKIP_PKG_CONFIG_GENERATION
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CNOID_INCLUDE_DIRS}
)

add_subdirectory(src)
