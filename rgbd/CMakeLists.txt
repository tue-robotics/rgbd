cmake_minimum_required(VERSION 2.8.3)
project(rgbd)

find_package(Boost REQUIRED)
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
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
)

# to show header files in Qt Creator
file(GLOB_RECURSE HEADER_FILES include/*.h)

add_library(rgbd    src/client.cpp
                    src/client_rgbd.cpp
                    src/client_ros.cpp
                    src/client_shm.cpp
                    src/serialization.cpp
                    src/server.cpp
                    src/server_rgbd.cpp
                    src/server_ros.cpp
                    src/server_shm.cpp
                    src/tools.cpp
                    src/view.cpp
                    src/ros/conversions.cpp

                    ${HEADER_FILES})
target_link_libraries(rgbd ${catkin_LIBRARIES} rt)
add_dependencies(rgbd ${catkin_EXPORTED_TARGETS})

# - - - - - - - - - - - - - - - - - NODES - - - - - - - - - - - - - - - - -

add_executable(ros_to_rgbd src/ros_to_rgbd.cpp)
target_link_libraries(ros_to_rgbd rgbd)

add_executable(rgbd_to_shm src/rgbd_to_shm.cpp)
target_link_libraries(rgbd_to_shm rgbd)

add_executable(rgbd_to_ros src/rgbd_to_ros.cpp)
target_link_libraries(rgbd_to_ros rgbd)

add_executable(get_3d_point_from_image_roi_node src/get_3d_point_from_image_roi_node.cpp)
target_link_libraries(get_3d_point_from_image_roi_node rgbd)

# - - - - - - - - - - - - - - - - - TOOLS - - - - - - - - - - - - - - - - -

add_executable(analyser tools/analyser.cpp)
target_link_libraries(analyser rgbd)

add_executable(full_screen_viewer tools/full_screen_viewer.cpp)
target_link_libraries(full_screen_viewer rgbd)

add_executable(multitool tools/multitool.cpp)
target_link_libraries(multitool rgbd)

add_executable(record_to_video tools/record_to_video.cpp)
target_link_libraries(record_to_video rgbd)

add_executable(rgbd_to_rgb_png tools/rgbd_to_rgb_png.cpp)
target_link_libraries(rgbd_to_rgb_png rgbd)

add_executable(save tools/save.cpp)
target_link_libraries(save rgbd)

add_executable(view tools/view.cpp)
target_link_libraries(view rgbd)

add_executable(viewer tools/viewer.cpp)
target_link_libraries(viewer rgbd)

# - - - - - - - - - - - - - - - - - TESTS - - - - - - - - - - - - - - - - -

add_executable(rgbd_test_server test/test_server.cpp test/test_server_templ.h)
target_link_libraries(rgbd_test_server rgbd)

add_executable(rgbd_test_server_rgbd test/test_server_rgbd.cpp test/test_server_templ.h)
target_link_libraries(rgbd_test_server_rgbd rgbd)

add_executable(rgbd_test_server_shm test/test_server_shm.cpp test/test_server_templ.h)
target_link_libraries(rgbd_test_server_shm rgbd)

add_executable(rgbd_test_client test/test_client.cpp test/test_client_templ.h)
target_link_libraries(rgbd_test_client rgbd)

add_executable(rgbd_test_client_rgbd test/test_client_rgbd.cpp test/test_client_templ.h)
target_link_libraries(rgbd_test_client_rgbd rgbd)

add_executable(rgbd_test_client_shm test/test_client_shm.cpp test/test_client_templ.h)
target_link_libraries(rgbd_test_client_shm rgbd)

add_executable(rgbd_test_serialization test/test_serialization.cpp)
target_link_libraries(rgbd_test_serialization rgbd)

# - - - - - - - - - - - - - - - - - - CI - - - - - - - - - - - - - - - - -

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(serialization_gtest test/serialization_gtest.cpp)
  target_link_libraries(serialization_gtest rgbd ${catkin_LIBRARIES})
endif()
