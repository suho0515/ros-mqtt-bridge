cmake_minimum_required(VERSION 3.0.2)
project(ips)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Tag.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ips
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS scripts/ips_node.py scripts/pose_estimator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
