yarp_ros2
---------

This is a work in progress.

This repository contains the YARP devices and utilities for ROS2.

### Build with ROS msgs compiled in separate colcon workspace

~~~bash
# Compile the colcon workspace containing the required messages and services
(cd ros2_interfaces_ws && colcon build --packages-select map2d_nws_ros2_msgs)

# Make the workspace available
. ros2_interfaces_ws/install/setup.bash

# Configure and compile
cmake -S. -Bbuild
cmake --build build
~~~

### Build with pure CMake commands

~~~
# Configure, compile and install
cmake -S. -Bbuild -DCMAKE_INSTALL_PREFIX=<install_prefix>
cmake --build build
cmake --build build --target install

# Make ROS msgs available in [ament index](https://github.com/ament/ament_index)
export AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH:<install_prefix>
~~~

