cmake_minimum_required(VERSION 3.0.2)
project(rgbd_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  message_generation
)

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
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs
)
