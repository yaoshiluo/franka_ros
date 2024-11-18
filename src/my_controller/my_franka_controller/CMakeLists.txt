cmake_minimum_required(VERSION 3.0.2)
project(my_franka_controller)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_controller
  CATKIN_DEPENDS controller_interface hardware_interface pluginlib roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(my_controller src/my_controller.cpp)
target_link_libraries(my_controller ${catkin_LIBRARIES})
add_dependencies(my_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS my_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

