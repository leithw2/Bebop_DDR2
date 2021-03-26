# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/local/include/eigen3".split(';') if "${prefix}/include;/usr/local/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "geometry_msgs;mav_msgs;nav_msgs;roscpp;sensor_msgs;std_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lposition_controller;-lextendedKalmanFilter;-lposition_controller_with_bebop;-lwaypoint_filter".split(';') if "-lposition_controller;-lextendedKalmanFilter;-lposition_controller_with_bebop;-lwaypoint_filter" != "" else []
PROJECT_NAME = "bebop_simulator"
PROJECT_SPACE_DIR = "/home/laptop/catkin_ws/install"
PROJECT_VERSION = "1.0.2"
