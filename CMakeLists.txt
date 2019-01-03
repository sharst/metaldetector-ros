cmake_minimum_required(VERSION 2.8.3)
project(metaldetector_ros)

find_package(catkin REQUIRED
  COMPONENTS
    can_msgs
    socketcan_interface
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    can_msgs
    socketcan_interface
)


install(
  PROGRAMS
    scripts/metalnode
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


