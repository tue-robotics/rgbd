cmake_minimum_required(VERSION 2.8.3)
project(rgbd_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  message_generation
)

add_definitions(-std=c++11)

add_message_files(
  FILES
    RGBD.msg
)

add_service_files(
  FILES
    GetRGBD.srv
    Project2DTo3D.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs std_msgs
)
