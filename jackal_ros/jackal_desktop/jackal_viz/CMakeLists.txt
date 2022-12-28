cmake_minimum_required(VERSION 2.8.3)
project(jackal_viz)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch/view_model.launch)
roslaunch_add_file_check(launch/view_robot.launch)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
