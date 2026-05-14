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
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${OpenCV_LIBRARIES}
    rclcpp::rclcpp
    rgbd_interfaces::rgbd_interfaces__rosidl_typesupport_cpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
    rt
)
ament_target_dependencies(${PROJECT_NAME}
  cv_bridge
  geolib2
  geometry_msgs
  image_geometry
  message_filters
  pcl_conversions
  pcl_ros
  rgbd_interfaces
  sensor_msgs
  std_msgs
  tf2
  tue_serialization
)

add_executable(ros_to_${PROJECT_NAME} src/ros_to_rgbd.cpp)
target_link_libraries(ros_to_${PROJECT_NAME}
  ${PROJECT_NAME}
)

add_library(ros_to_${PROJECT_NAME}_component SHARED src/ros_to_rgbd_component.cpp)
target_link_libraries(ros_to_${PROJECT_NAME}_component ${PROJECT_NAME})
ament_target_dependencies(ros_to_${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
)
rclcpp_components_register_nodes(ros_to_${PROJECT_NAME}_component "rgbd::RosToRGBDComponent")
target_link_libraries(ros_to_${PROJECT_NAME} ros_to_${PROJECT_NAME}_component)

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
    get_3d_point_from_image_roi_node
    ros_to_${PROJECT_NAME}_component
    ${PROJECT_NAME}_to_ros
    ${PROJECT_NAME}_to_shm
    ros_to_${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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
