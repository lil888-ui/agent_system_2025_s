cmake_minimum_required(VERSION 3.5)
project(choreonoid_ros_tutorial)

# Catkin と必要なコンポーネントのみをここで読み込む
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Choreonoid本体の設定はルートCMakeLists.txtで行うため、ここではインクルードのみ
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CNOID_INCLUDE_DIRS}
)

# スマートコントローラを登録
choreonoid_add_simple_controller(
  RttTankController
  RttTankController.cpp
)

# ROSおよびChoreonoidのライブラリをリンク
target_link_libraries(RttTankController
  ${catkin_LIBRARIES}
  ${CNOID_LIBRARIES}
)
