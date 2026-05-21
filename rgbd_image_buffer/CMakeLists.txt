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
  find_package(tue_lint_config REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_clang_tidy REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  
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
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(geolib2 geometry_msgs rclcpp rgbd tf2 tf2_ros)

ament_package()
