# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "controller_interface;hardware_interface;pluginlib;roscpp".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lmy_controller".split(';') if "-lmy_controller" != "" else []
PROJECT_NAME = "my_franka_controller"
PROJECT_SPACE_DIR = "/home/ubuntu/franka_ros_ws/install"
PROJECT_VERSION = "0.0.0"
