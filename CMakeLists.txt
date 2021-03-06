cmake_minimum_required(VERSION 2.8.3)
project(carl_moveit)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cmake_modules
  control_msgs
  message_generation
  geometry_msgs
  roscpp
  moveit_ros_planning_interface
  pcl_conversions
  pcl_ros
  rail_manipulation_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  wpi_jaco_msgs
)

find_package(Eigen REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate actions in the 'action' folder
add_action_files(
  FILES
  WipeSurface.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also nee
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(carl_moveit_wrapper src/carl_moveit.cpp include/carl_moveit/carl_moveit.h)
add_executable(tablebot_common_actions src/common_actions.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(carl_moveit_wrapper ${catkin_LIBRARIES})
target_link_libraries(tablebot_common_actions ${catkin_LIBRARIES})

add_dependencies(carl_moveit_wrapper rail_manipulation_msgs_gencpp)
add_dependencies(tablebot_common_actions carl_moveit_gencpp rail_manipulation_msgs_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS carl_moveit_wrapper tablebot_common_actions
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Copy config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
