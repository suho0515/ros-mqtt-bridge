cmake_minimum_required(VERSION 3.0.2)
project(joy_to_cmd_converter)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES joy_to_cmd_converter
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/joy_to_cmd_converter_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)