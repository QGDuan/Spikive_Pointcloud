cmake_minimum_required(VERSION 3.0.2)
project(spikive_pipeline)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rosbag
  sensor_msgs
  nav_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/drone_2_guangzhou_pipeline.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
