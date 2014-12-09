cmake_minimum_required(VERSION 2.8.3)
project(rgbd)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geolib2
  image_geometry
  message_filters
  pcl_conversions
  pcl_ros
  roscpp
  tue_serialization
  message_generation
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  RGBDMsg.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rgbd
  CATKIN_DEPENDS cv_bridge geolib2 image_geometry message_filters pcl_conversions pcl_ros roscpp tue_serialization
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

message("include dirs: " + ${catkin_INCLUDE_DIRS})
message("libs: " + ${catkin_LIBRARIES})

# to show header files in Qt Creator
file(GLOB_RECURSE HEADER_FILES include/*.h)

add_library(rgbd    src/Server.cpp
                    src/Client.cpp
                    src/ClientRaw.cpp
                    src/Image.cpp
                    src/View.cpp
                    src/serialization.cpp
                    ${HEADER_FILES})

target_link_libraries(rgbd ${catkin_LIBRARIES})

add_executable(rgbd_server src/server.cpp)
target_link_libraries(rgbd_server rgbd)
add_dependencies(rgbd_server rgbd_generate_messages_cpp)

add_executable(rgbd_viewer src/viewer.cpp)
target_link_libraries(rgbd_viewer rgbd)
add_dependencies(rgbd_viewer rgbd_generate_messages_cpp)

add_executable(rgbd_save src/save.cpp)
target_link_libraries(rgbd_save rgbd)
add_dependencies(rgbd_save rgbd_generate_messages_cpp)

add_executable(rgbd_view src/view.cpp)
target_link_libraries(rgbd_view rgbd)
add_dependencies(rgbd_view rgbd_generate_messages_cpp)

add_executable(rgbd_test_client test/test_client.cpp)
target_link_libraries(rgbd_test_client rgbd)
add_dependencies(rgbd_test_client rgbd_generate_messages_cpp)

add_executable(rgbd_test_serialization test/test_serialization.cpp)
target_link_libraries(rgbd_test_serialization rgbd)
add_dependencies(rgbd_test_serialization rgbd_generate_messages_cpp)

