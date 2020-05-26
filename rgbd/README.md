# rgbd
[![Build Status](https://travis-ci.org/tue-robotics/rgbd.svg?branch=master)](https://travis-ci.org/tue-robotics/rgbd)

The rgbd server reads data from a 3D sensor and publishes it in a compressed form.
It provides a unified format for depthmaps combined with RGB data, which is more compact than a point cloud.

## Core classes
All **bold names** are classes. The main interface types are shown to understand the differences between all classes.
![core classes](doc/rgbd_classes.svg)

## Typical usage
### Classes
For client applications, i.e., applications consuming RGBD data, it is advised to use the `Client` class. If possible, this will obtain its images from shared memory using the `ClientSHM` class, which minimizes network and CPU usage. Otherwise it relies on data received as `rgbd_msgs::RGBD` messages through the `ClientRGBD` class. Data is typically obtained as instance of the `rgbd::Image` class using the `nextImage` functions (see docstrings for more info).

The `ClientROS` class is present in case no data is provided in RGBD format. This is used, e.g., in the `ros_to_rgbd` node (see below).

The `Server` class does exactly the opposite from the `Client` class. It receives an instance of an `rbgd::Image` and both (re-)sends this data as an `rgbd_msgs::RGBD` message over a ROS topic and writes it to shared memory.

### Nodes
This package contains a number of nodes and tools. The most important ones are:
* `rgbd_to_shm`: contains an instance of `ClientRGBD` that listens to RGBD data received over a `rgbd_msgs::RGBD` topic and writes this to shared memory using an instance of a `ServerSHM`. By running this node, only one incoming connection for the `rgbd` messages is setup since all clients can subsequently use the shared memory. If your robot contains multiple computers, it is advised to run this node on each computer where (more than 1) `Client` instances are used.
* `ros_to_rgbd`: converts 'standard' ROS messages (`sensor_msgs::Image` (color image and depth image) and ` sensor_msgs::CameraInfo`) to the `rgbd_msgs::RGBD` format. Hereto, it uses instances of the `ClientROS` and the `Server` classes.
* `rgbd_to_ros`: converts `rgbd_msgs::RGBD` messages to the 'standard' ROS format (`sensor_msgs::Image` (color image and depth image) and `sensor_msgs::CameraInfo`). Hereto, it uses an instance of the `Client` class as well as publishers for the RGB image, RGB camera info, depth image, depth camera info and pointcloud.

## Notes
In case you are developing camera drivers, it is advised to stick to the 'standard' ROS interfaces to minimize dependencies which enhances compatibility with other ROS users. Use the `ros_to_rgbd` node and, if applicable, the `rgbd_to_shm` server to interface with your software if that's using the `Client` class.
