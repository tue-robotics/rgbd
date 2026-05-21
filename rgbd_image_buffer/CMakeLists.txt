cmake_minimum_required(VERSION 3.8)
project(rgbd_image_buffer)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(ament_cmake REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(geolib2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rgbd REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# to show header files in Qt Creator
file(GLOB_RECURSE HEADER_FILES include/*.h)

add_library(${PROJECT_NAME}
  ${HEADER_FILES}
  src/image_buffer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  geolib2::geolib
  rclcpp::rclcpp
  tf2::tf2
)
if(TARGET rgbd::rgbd)
  target_link_libraries(${PROJECT_NAME} PUBLIC rgbd::rgbd)
else()
  target_link_libraries(${PROJECT_NAME} PUBLIC rgbd)
endif()
if(TARGET tf2_ros::tf2_ros)
  target_link_libraries(${PROJECT_NAME} PUBLIC tf2_ros::tf2_ros)
else()
  target_include_directories(${PROJECT_NAME} PUBLIC ${tf2_ros_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${tf2_ros_LIBRARIES})
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(geolib2 geometry_msgs rclcpp rgbd tf2 tf2_ros)

ament_package()
