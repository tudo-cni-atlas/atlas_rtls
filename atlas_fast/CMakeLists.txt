cmake_minimum_required(VERSION 2.8.3)
project(atlas_fast)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  atlas_msgs
  roslib
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

## Activate C++11 support
add_compile_options(-std=c++11)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/tags_dynParams.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs atlas_msgs actionlib_msgs actionlib dynamic_reconfigure
  DEPENDS system_lib Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

## Declare a C++ executable
SET(SOURCES
  src/Scheduler.cpp
)
add_executable(atlas_fast_node src/main.cpp ${SOURCES})
target_link_libraries(
  atlas_fast_node
  ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(atlas_fast_node ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(atlas_fast_node ${catkin_LIBRARIES})