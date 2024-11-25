cmake_minimum_required(VERSION 3.4)
project(my_impedance_controller)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_hw
  geometry_msgs
  hardware_interface
  message_generation
  pluginlib
  realtime_tools
  roscpp
  urdf
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

add_message_files(FILES
  JointTorqueComparison.msg
)
# 生成动态重配置文件
generate_dynamic_reconfigure_options(
  cfg/compliance_param.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_impedance_controller
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    eigen_conversions
    franka_hw
    geometry_msgs
    hardware_interface
    message_runtime
    pluginlib
    realtime_tools
    roscpp
    urdf
    visualization_msgs
  DEPENDS Franka
)

# 添加库
add_library(my_impedance_controller
  src/cartesian_impedance_controller.cpp
)

add_dependencies(my_impedance_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  # ${PROJECT_NAME}_generate_messages_cpp
  # ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(my_impedance_controller PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(my_impedance_controller SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(my_impedance_controller PUBLIC
  include
)


# 安装部分
install(TARGETS my_impedance_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES my_impedance_controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
