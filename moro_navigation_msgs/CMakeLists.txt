cmake_minimum_required(VERSION 2.8.3)
project(moro_navigation_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  trajectory_msgs
  # actionlib_msgs
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  PlanPath.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Navigate.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs trajectory_msgs
)
