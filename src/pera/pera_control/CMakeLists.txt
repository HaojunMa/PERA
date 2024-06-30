cmake_minimum_required(VERSION 2.8.3)
project(pera_control)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

install(DIRECTORY demos
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS bin/pera_driver
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
