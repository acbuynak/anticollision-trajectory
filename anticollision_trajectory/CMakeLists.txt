cmake_minimum_required(VERSION 3.0.2)

project(anticollision_trajectory)

find_package(catkin REQUIRED)

######################################
## Catkin Specific Configuration

catkin_package()

######################################
## Install

install(DIRECTORY launch scripts scripts_matlab
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
