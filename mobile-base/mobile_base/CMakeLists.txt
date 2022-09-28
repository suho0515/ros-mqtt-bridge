cmake_minimum_required(VERSION 3.0.2)
project(mobile_base)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ips
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS scripts/mobile_base_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)