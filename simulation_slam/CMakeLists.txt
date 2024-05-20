cmake_minimum_required(VERSION 3.0.2)
project(simulation_slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs 
  message_generation
)

add_message_files(
  FILES
  DualOccupancyGrid.msg
  NavigationState.msg
  OccupancyGridFlagged.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simulation_slam
#  CATKIN_DEPENDS roscpp rospy std_msgs
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
# scripts
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)