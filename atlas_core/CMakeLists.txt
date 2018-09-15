cmake_minimum_required(VERSION 2.8.3)
project(atlas_core)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  atlas_msgs
  roslib
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

## Activate C++11 support
add_compile_options(-std=c++11)

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
  CATKIN_DEPENDS roscpp rospy std_msgs atlas_msgs actionlib_msgs actionlib
  DEPENDS system_lib Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
SET(SOURCES
  src/Assembly.cpp
  src/ClockCorrection.cpp
  src/Reporter.cpp
)
add_executable(atlas_core_node src/main.cpp ${SOURCES})
target_link_libraries(
    atlas_core_node
    ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(atlas_core_node ${PROJECT_NAME}_gencfg)