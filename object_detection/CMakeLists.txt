cmake_minimum_required(VERSION 3.0.2)
project(object_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
)


catkin_package(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/object_tracking_static_pub.py scripts/object_tracking_dynamic_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)