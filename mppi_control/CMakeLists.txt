cmake_minimum_required(VERSION 3.0.2)
project(mppi_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs	
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

