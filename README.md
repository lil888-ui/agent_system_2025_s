自動走行実行


    rosrun choreonoid_ros_tutorial path_tracking.py \
      _csv_path:=/home/naya728/ros/agent_system_ws/src/choreonoid_ros_tutorial/src/log.csv \
      _distance_threshold:=0.05 \
      _angular_threshold:=0.1 \
      _linear_speed:=0.2 \
      _angular_speed:=0.5 \
      _rate:=10

a
    source ~/ros/agent_system_ws/devel/setup.bash

a
