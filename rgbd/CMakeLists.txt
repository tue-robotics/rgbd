cmake_minimum_required(VERSION 3.0.2)
project(rgbd)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geolib2
  geometry_msgs
  image_geometry
  message_filters
  nodelet
  pcl_ros
  pluginlib
  ${PROJECT_NAME}_msgs
  rosconsole_bridge
  roscpp
  sensor_msgs
  tue_serialization
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geolib2 image_geometry message_filters pcl_ros ${PROJECT_NAME}_msgs roscpp sensor_msgs tue_serialization
  DEPENDS Boost OpenCV
)

###########
## Build ##
###########

include_directories(
  include
  SYSTEM
  ${BOOST_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# to show header files in Qt Creator
file(GLOB_RECURSE HEADER_FILES include/*.h)

add_library(${PROJECT_NAME}
    ${HEADER_FILES}
    src/client.cpp
    src/client_${PROJECT_NAME}.cpp
    src/client_ros.cpp
    src/client_ros_base.cpp
    src/client_shm.cpp
    src/image.cpp
    src/ros/conversions.cpp
    # Forward geolib2 logging to rosconsole
    src/ros/rosconsole_bridge.cpp
    src/serialization.cpp
    src/server.cpp
    src/server_${PROJECT_NAME}.cpp
    src/server_ros.cpp
    src/server_shm.cpp
    src/utility.cpp
    src/view.cpp
)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} rt)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# - - - - - - - - - - - - - - - - NODELETS - - - - - - - - - - - - - - - -

add_library(${PROJECT_NAME}_nodelets src/nodelets/ros_to_${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME}_nodelets ${PROJECT_NAME} ${catkin_LIBRARIES})

# - - - - - - - - - - - - - - - - - NODES - - - - - - - - - - - - - - - - -

add_executable(ros_to_${PROJECT_NAME} src/ros_to_${PROJECT_NAME}.cpp)
target_link_libraries(ros_to_${PROJECT_NAME} ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_to_shm src/rgbd_to_shm.cpp)
target_link_libraries(${PROJECT_NAME}_to_shm ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_to_ros src/rgbd_to_ros.cpp)
target_link_libraries(${PROJECT_NAME}_to_ros ${PROJECT_NAME})

add_executable(get_3d_point_from_image_roi_node src/get_3d_point_from_image_roi_node.cpp)
target_link_libraries(get_3d_point_from_image_roi_node ${PROJECT_NAME})

# - - - - - - - - - - - - - - - - - TOOLS - - - - - - - - - - - - - - - - -

add_executable(analyser tools/analyser.cpp)
target_link_libraries(analyser ${PROJECT_NAME})

add_executable(full_screen_viewer tools/full_screen_viewer.cpp)
target_link_libraries(full_screen_viewer ${PROJECT_NAME} ${OpenCV_LIBRARIES})

add_executable(multitool tools/multitool.cpp)
target_link_libraries(multitool ${PROJECT_NAME} ${OpenCV_LIBRARIES})

add_executable(record_to_video tools/record_to_video.cpp)
target_link_libraries(record_to_video ${PROJECT_NAME} ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME}_to_png tools/rgbd_to_png.cpp)
target_link_libraries(${PROJECT_NAME}_to_png ${PROJECT_NAME} ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME}_to_pcd tools/rgbd_to_pcd.cpp)
target_link_libraries(${PROJECT_NAME}_to_pcd ${PROJECT_NAME} ${OpenCV_LIBRARIES})

add_executable(save tools/save.cpp)
target_link_libraries(save ${PROJECT_NAME})

add_executable(view tools/view.cpp)
target_link_libraries(view ${PROJECT_NAME} ${OpenCV_LIBRARIES})

add_executable(viewer tools/viewer.cpp)
target_link_libraries(viewer ${PROJECT_NAME} ${OpenCV_LIBRARIES})

# - - - - - - - - - - - - - - - - - TESTS - - - - - - - - - - - - - - - - -

add_library(test_server_templ OBJECT test/test_server_templ.h)
target_link_libraries(test_server_templ ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_test_server test/test_server.cpp)
target_link_libraries(${PROJECT_NAME}_test_server $<TARGET_OBJECTS:test_server_templ> ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_test_server_${PROJECT_NAME} test/test_server_rgbd.cpp)
target_link_libraries(${PROJECT_NAME}_test_server_${PROJECT_NAME} $<TARGET_OBJECTS:test_server_templ> ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_test_server_shm test/test_server_shm.cpp)
target_link_libraries(${PROJECT_NAME}_test_server_shm $<TARGET_OBJECTS:test_server_templ> ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_test_server_ros test/test_server_ros.cpp)
target_link_libraries(${PROJECT_NAME}_test_server_ros $<TARGET_OBJECTS:test_server_templ> ${PROJECT_NAME})

add_library(test_client_templ OBJECT test/test_client_templ.h)
target_link_libraries(test_client_templ ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_test_client test/test_client.cpp)
target_link_libraries(${PROJECT_NAME}_test_client $<TARGET_OBJECTS:test_client_templ> ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_test_client_${PROJECT_NAME} test/test_client_${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME}_test_client_${PROJECT_NAME} $<TARGET_OBJECTS:test_client_templ> ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_test_client_shm test/test_client_shm.cpp)
target_link_libraries(${PROJECT_NAME}_test_client_shm $<TARGET_OBJECTS:test_client_templ> ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_test_client_ros test/test_client_ros.cpp)
target_link_libraries(${PROJECT_NAME}_test_client_ros $<TARGET_OBJECTS:test_client_templ> ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_test_serialization test/test_serialization.cpp)
target_link_libraries(${PROJECT_NAME}_test_serialization ${OpenCV_LIBRARIES} ${PROJECT_NAME})

#############
## Install ##
#############

install(
  FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_nodelets
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  TARGETS
    analyser
    full_screen_viewer
    get_3d_point_from_image_roi_node
    multitool
    record_to_video
    ${PROJECT_NAME}_test_client
    ${PROJECT_NAME}_test_client_${PROJECT_NAME}
    ${PROJECT_NAME}_test_client_ros
    ${PROJECT_NAME}_test_client_shm
    ${PROJECT_NAME}_test_server
    ${PROJECT_NAME}_test_server_${PROJECT_NAME}
    ${PROJECT_NAME}_test_server_ros
    ${PROJECT_NAME}_test_server_shm
    ${PROJECT_NAME}_to_pcd
    ${PROJECT_NAME}_to_png
    ${PROJECT_NAME}_to_ros
    ${PROJECT_NAME}_to_shm
    ros_to_${PROJECT_NAME}
    save
    view
    viewer
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(catkin_lint_cmake REQUIRED)
  catkin_add_catkin_lint_test(-W2)

  include_directories(BEFORE SYSTEM ${GTEST_INCLUDE_DIRS})

  add_library(test_utils OBJECT test/test_utils.h)
  target_link_libraries(test_utils ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

  catkin_add_gtest(image_gtest test/image_gtest.cpp)
  target_link_libraries(image_gtest $<TARGET_OBJECTS:test_utils> ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

  catkin_add_gtest(serialization_gtest test/serialization_gtest.cpp)
  target_link_libraries(serialization_gtest $<TARGET_OBJECTS:test_utils> ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

  add_executable(shm_server_gtest EXCLUDE_FROM_ALL test/shm_server_gtest.cpp)
  target_link_libraries(shm_server_gtest $<TARGET_OBJECTS:test_utils> ${PROJECT_NAME} ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
  add_dependencies(tests shm_server_gtest)
  add_rostest(test/shm_server.test)

  add_executable(connection_${PROJECT_NAME}_gtest EXCLUDE_FROM_ALL test/connection_rgbd_gtest.cpp)
  target_link_libraries(connection_${PROJECT_NAME}_gtest $<TARGET_OBJECTS:test_utils> ${PROJECT_NAME} ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
  add_dependencies(tests connection_${PROJECT_NAME}_gtest)
  add_rostest(test/connection_${PROJECT_NAME}.test)

  add_executable(connection_ros_gtest EXCLUDE_FROM_ALL test/connection_ros_gtest.cpp)
  target_link_libraries(connection_ros_gtest $<TARGET_OBJECTS:test_utils> ${PROJECT_NAME} ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
  add_dependencies(tests connection_ros_gtest)
  add_rostest(test/connection_ros.test)

  add_executable(connection_shm_gtest EXCLUDE_FROM_ALL test/connection_shm_gtest.cpp)
  target_link_libraries(connection_shm_gtest $<TARGET_OBJECTS:test_utils> ${PROJECT_NAME} ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
  add_dependencies(tests connection_shm_gtest)
  add_rostest(test/connection_shm.test)

  add_executable(connection_gtest EXCLUDE_FROM_ALL test/connection_gtest.cpp)
  target_link_libraries(connection_gtest ${PROJECT_NAME} ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
  add_dependencies(tests connection_gtest)
  add_rostest(test/connection.test)
endif()
