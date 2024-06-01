cmake_minimum_required(VERSION 3.0.2)
project(move_panda)

find_package(catkin REQUIRED COMPONENTS

geometry_msgs
image_transport
message_generation
roscpp
rospy
sensor_msgs
std_msgs
tf2
tf2_ros
)



add_message_files(
  FILES
  PredictedPoses.msg
)


add_executable(static_egrocery_task src/static_egrocery_task.cpp)
target_link_libraries(static_egrocery_task ${catkin_LIBRARIES})
target_include_directories(static_egrocery_task SYSTEM PUBLIC
)




generate_messages(
  DEPENDENCIES
  geometry_msgs   sensor_msgs   std_msgs
)

catkin_package(
 CATKIN_DEPENDS  geometry_msgs image_transport message_runtime roscpp rospy sensor_msgs std_msgs tf2 tf2_ros
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
