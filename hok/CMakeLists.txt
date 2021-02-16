cmake_minimum_required(VERSION 2.8.3)
project(hok)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hok
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

 install(PROGRAMS
   scripts/listener.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
