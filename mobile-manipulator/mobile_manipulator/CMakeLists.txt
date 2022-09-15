cmake_minimum_required(VERSION 3.0.2)
project(mobile_manipulator)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/ur_node.py src/mobile_manipulator.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})