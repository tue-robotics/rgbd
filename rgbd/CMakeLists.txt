cmake_minimum_required(VERSION 3.8)
project(rgbd)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geolib2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rgbd_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tue_serialization REQUIRED)

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
  src/serialization.cpp
  src/server.cpp
  src/server_${PROJECT_NAME}.cpp
  src/server_ros.cpp
  src/server_shm.cpp
  src/utility.cpp
  src/view.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${IMAGE_GEOMETRY_INCLUDE_DIR}
  ${MESSAGE_FILTERS_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  opencv_core
  cv_bridge::cv_bridge
  geolib2::geolib
  image_geometry::image_geometry
  message_filters::message_filters
  rclcpp::rclcpp
  tf2::tf2
  rgbd_interfaces::rgbd_interfaces__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  std_msgs::std_msgs__rosidl_typesupport_cpp
  rt
)
if(TARGET pcl_conversions::pcl_conversions)
  target_link_libraries(${PROJECT_NAME} PUBLIC pcl_conversions::pcl_conversions)
else()
  target_include_directories(${PROJECT_NAME} PUBLIC ${pcl_conversions_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${pcl_conversions_LIBRARIES})
endif()
target_link_libraries(${PROJECT_NAME} PUBLIC ${pcl_ros_LIBRARIES})
if(TARGET tue_serialization::tue_serialization)
  target_link_libraries(${PROJECT_NAME} PUBLIC tue_serialization::tue_serialization)
else()
  target_include_directories(${PROJECT_NAME} PUBLIC ${tue_serialization_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${tue_serialization_LIBRARIES})
endif()

add_library(ros_to_${PROJECT_NAME}_component SHARED
  src/ros_to_rgbd_component.cpp
  src/ros_to_rgbd_component_registration.cpp
)
target_link_libraries(ros_to_${PROJECT_NAME}_component
  ${PROJECT_NAME}
  rclcpp::rclcpp
  rclcpp_components::component
)
rclcpp_components_register_nodes(ros_to_${PROJECT_NAME}_component
  PLUGIN "rgbd::RosToRGBDComponent"
  EXECUTABLE ros_to_${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_to_shm src/rgbd_to_shm.cpp)
target_link_libraries(${PROJECT_NAME}_to_shm ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_to_ros src/rgbd_to_ros.cpp)
target_link_libraries(${PROJECT_NAME}_to_ros ${PROJECT_NAME})

add_executable(get_3d_point_from_image_roi_node src/get_3d_point_from_image_roi_node.cpp)
target_link_libraries(get_3d_point_from_image_roi_node ${PROJECT_NAME})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS
    ${PROJECT_NAME}
    ros_to_${PROJECT_NAME}_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    get_3d_point_from_image_roi_node
    ${PROJECT_NAME}_to_ros
    ${PROJECT_NAME}_to_shm
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(tue_lint_config REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_clang_tidy REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_clang_format(
    CONFIG_FILE ${tue_lint_config_DIR}/../config/.clang-format
    --clang-format-version=21
  )
  ament_clang_tidy(
    CONFIG_FILE ${tue_lint_config_DIR}/../config/.clang-tidy
    --clang-tidy-version=21
    ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_BINARY_DIR}/compile_commands.json)
  ament_lint_cmake(MAX_LINE_LENGTH 120 "--filter=
    +convention/filename,
    +linelength,
    +package/consistency,
    +package/stdargs,
    +readability/logic,
    +readability/mixedcase,
    +readability/wonkycase,
    +syntax,+whitespace/eol,
    +whitespace/extra,
    +whitespace/indent,
    +whitespace/mismatch,
    +whitespace/newline,
    +whitespace/tabs"
  )
  ament_xmllint(MAX_LINE_LENGTH 120)

  ament_add_gtest(image_gtest test/image_gtest.cpp)
  target_link_libraries(image_gtest ${PROJECT_NAME} ${OpenCV_LIBRARIES})

  ament_add_gtest(serialization_gtest test/serialization_gtest.cpp)
  target_link_libraries(serialization_gtest ${PROJECT_NAME} ${OpenCV_LIBRARIES})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  cv_bridge
  geolib2
  geometry_msgs
  image_geometry
  message_filters
  pcl_conversions
  pcl_ros
  rclcpp_components
  rgbd_interfaces
  sensor_msgs
  std_msgs
  tf2
  tue_serialization
)

ament_package()
