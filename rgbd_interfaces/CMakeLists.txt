cmake_minimum_required(VERSION 3.8)
project(rgbd_interfaces)

find_package(ament_cmake REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RGBD.msg"
  "srv/GetRGBD.srv"
  "srv/Project2DTo3D.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
