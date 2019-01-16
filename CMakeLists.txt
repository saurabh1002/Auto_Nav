cmake_minimum_required(VERSION 2.8.3)
project(Auto_Nav)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  move_base
  skeleton
  sweep_ros
  amcl
  rosserial_python
  map_server
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
