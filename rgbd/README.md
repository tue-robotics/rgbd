# rgbd
[![Build Status](https://travis-ci.org/tue-robotics/rgbd.svg?branch=master)](https://travis-ci.org/tue-robotics/rgbd)

The rgbd server reads data from a 3D sensor and publishes it in a compressed form.
It provides a unified format for depthmaps combined with RGB data, which is more compact than a point cloud.

## Core classes
All **bold names** are classes. The main interface types are shown to understand the differences between all classes.
![core classes](doc/rgbd_classes.svg)