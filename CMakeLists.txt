cmake_minimum_required(VERSION 2.8.3)
project(mobrob_robotcode)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  robot_state_publisher
  rviz
  xacro  
  roscpp
  rospy
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mobrob_robotcode
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)


find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  tf
  myrobot_model
)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_state_publisher src/state_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_state_publisher ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_state_publisher ${catkin_EXPORTED_TARGETS})


add_executable(${PROJECT_NAME}_odom_converter src/odom_converter.cpp)
target_link_libraries(${PROJECT_NAME}_odom_converter ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_odom_converter ${catkin_EXPORTED_TARGETS})
