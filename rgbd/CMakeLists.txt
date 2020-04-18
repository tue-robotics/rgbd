cmake_minimum_required(VERSION 2.8.3)
project(rgbd)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geolib2
  geometry_msgs
  image_geometry
  message_filters
  pcl_ros
  rgbd_msgs
  roscpp
  sensor_msgs
  tue_serialization
)

add_compile_options(-std=c++11)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rgbd
  CATKIN_DEPENDS cv_bridge geolib2 image_geometry message_filters pcl_ros rgbd_msgs roscpp sensor_msgs tue_serialization
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# to show header files in Qt Creator
file(GLOB_RECURSE HEADER_FILES include/*.h)

add_library(rgbd    src/Server.cpp
                    src/Server_RGBD.cpp
                    src/ServerShmOnly.cpp
                    src/Client.cpp
                    src/Client_ROS.cpp
                    src/View.cpp
                    src/serialization.cpp
                    src/ros/conversions.cpp

                    src/shared_mem_server.cpp
                    src/shared_mem_client.cpp

                    ${HEADER_FILES})
target_link_libraries(rgbd ${catkin_LIBRARIES} rt)

add_dependencies(rgbd ${catkin_EXPORTED_TARGETS})

# - - - - - - - - - - - - - - - - - TOOLS - - - - - - - - - - - - - - - - -

add_executable(rgbd_server src/server.cpp)
target_link_libraries(rgbd_server rgbd)

add_executable(rgbd_to_shm src/rgbd_to_shm.cpp)
target_link_libraries(rgbd_to_shm rgbd)

add_executable(rgbd_viewer src/viewer.cpp)
target_link_libraries(rgbd_viewer rgbd)

add_executable(rgbd_to_ros src/rgbd_to_ros.cpp)
target_link_libraries(rgbd_to_ros rgbd)

add_executable(get_3d_point_from_image_roi_node src/get_3d_point_from_image_roi_node.cpp)
target_link_libraries(get_3d_point_from_image_roi_node rgbd)

add_executable(rgbd_save src/save.cpp)
target_link_libraries(rgbd_save rgbd)

add_executable(rgbd_view src/view.cpp)
target_link_libraries(rgbd_view rgbd)

add_executable(record_to_video tools/record_to_video.cpp)
target_link_libraries(record_to_video rgbd)

add_executable(multitool tools/multitool.cpp)
target_link_libraries(multitool rgbd)

add_executable(presenter tools/presenter.cpp)
target_link_libraries(presenter rgbd)

add_executable(rgbd_analyser tools/analyser.cpp)
target_link_libraries(rgbd_analyser rgbd)

add_executable(rgbd_to_rgb_png tools/rgbd_to_rgb_png.cpp)
target_link_libraries(rgbd_to_rgb_png rgbd)

add_executable(full_screen_viewer src/full_screen_viewer.cpp)
target_link_libraries(full_screen_viewer rgbd)

# - - - - - - - - - - - - - - - - - TESTS - - - - - - - - - - - - - - - - -

add_executable(rgbd_test_server test/test_server.cpp)
target_link_libraries(rgbd_test_server rgbd)

add_executable(rgbd_test_client test/test_client.cpp)
target_link_libraries(rgbd_test_client rgbd)

add_executable(rgbd_test_serialization test/test_serialization.cpp)
target_link_libraries(rgbd_test_serialization rgbd)

