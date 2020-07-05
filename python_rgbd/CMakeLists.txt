cmake_minimum_required(VERSION 2.4.6)
if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()

project(python_rgbd)

find_package(rgbd REQUIRED)
include_directories(${rgbd_INCLUDE_DIRS})
link_directories(${rgbd_LIBRARY_DIRS})

if(DEFINED ENV{ROS_PYTHON_VERSION})
  SET(PYTHON_VERSION $ENV{ROS_PYTHON_VERSION} CACHE STRING "Python Version")
else()
  SET(PYTHON_VERSION 2 CACHE STRING "Python Version")
endif()

find_package(PythonInterp ${PYTHON_VERSION} REQUIRED)
find_package(PythonLibs ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR} REQUIRED)
# get_python_lib in python3 produces path which isn't in sys.path: https://bugs.launchpad.net/ubuntu/+source/python3-stdlib-extensions/+bug/1832215
# execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "from distutils.sysconfig import get_python_lib; print(get_python_lib(plat_specific=True, prefix=''))" OUTPUT_VARIABLE PYTHON_SITE_PACKAGES OUTPUT_STRIP_TRAILING_WHITESPACE)
set(PYTHON_SITE_PACKAGES_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages")
set(LIBRARY_NAME "PyRGBD")

SET(PYTHON_MODULE_EXTENSION ".so")
add_subdirectory(pybind11)
  pybind11_add_module(${LIBRARY_NAME}
    PyRGBD/PyRGBD.h
    PyRGBD/PyRGBD.cpp
    #PyRGBD/image.cpp
    #PyRGBD/client.cpp
    PyRGBD/ndarray_converter.h
    PyRGBD/ndarray_converter.cpp)
target_link_libraries(${LIBRARY_NAME} PRIVATE ${rgbd_LIBRARIES})
install(TARGETS ${LIBRARY_NAME} DESTINATION "${PYTHON_SITE_PACKAGES_INSTALL_DIR}")

find_package(catkin QUIET)
if(catkin_FOUND)
  catkin_add_env_hooks(python_rgbd_site_packages SHELLS bash tcsh zsh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
endif()
