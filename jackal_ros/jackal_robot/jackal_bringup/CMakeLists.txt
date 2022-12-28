cmake_minimum_required(VERSION 2.8.3)
project(jackal_bringup)

find_package(catkin REQUIRED)

catkin_package()
catkin_python_setup()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/install scripts/navsat_rtk_relay
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
