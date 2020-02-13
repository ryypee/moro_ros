cmake_minimum_required(VERSION 2.8.3)
project(moro_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  moro_navigation_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  actionlib
  move_base_msgs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  rospy moro_navigation_msgs nav_msgs geometry_msgs std_msgs actionlib move_base_msgs
)

catkin_install_python(PROGRAMS scripts/moro_planner scripts/moro_navigator
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test
  #  DEPENDENCIES ${catkin_EXPORTED_TARGETS}
  )
endif()