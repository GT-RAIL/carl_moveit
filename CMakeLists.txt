cmake_minimum_required(VERSION 2.8.3)
project(carl_moveit)

find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs message_generation geometry_msgs cmake_modules roscpp moveit_ros_planning_interface sensor_msgs)
find_package(Eigen REQUIRED)

add_service_files(
  FILES
  CallIK.srv
)

add_action_files(
  FILES
  MoveToPose.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(carl_moveit_wrapper src/carl_moveit.cpp)
target_link_libraries(carl_moveit_wrapper ${catkin_LIBRARIES})
add_dependencies(carl_moveit_wrapper carl_moveit_gencpp)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
