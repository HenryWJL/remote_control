cmake_minimum_required(VERSION 3.0.2)
project(remote_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS
    scripts/server.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
