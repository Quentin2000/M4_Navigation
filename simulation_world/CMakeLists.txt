cmake_minimum_required(VERSION 3.0.2)
project(simulation_world)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  HipPos.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simulation_world
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS
  script/base_footprint_pub.py
  script/key_controller.py
  script/robot_mode_switch.py
  script/robot_mode_switch_rl.py
  script/wheel_velocity_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)