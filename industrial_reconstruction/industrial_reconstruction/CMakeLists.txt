cmake_minimum_required(VERSION 3.0.2)
project(industrial_reconstruction)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

catkin_install_python(
  PROGRAMS nodes/industrial_reconstruction_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
