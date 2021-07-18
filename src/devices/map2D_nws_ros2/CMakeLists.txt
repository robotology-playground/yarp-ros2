# Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

find_package(nav_msgs REQUIRED)
find_package(test_msgs REQUIRED)
if(YARP_ROS2_USE_SYSTEM_map2d_nws_ros2_msgs)
  find_package(map2d_nws_ros2_msgs REQUIRED)
endif()
find_package(visualization_msgs REQUIRED)

yarp_prepare_plugin(map2D_nws_ros2
  CATEGORY device
  TYPE Map2D_nws_ros2
  INCLUDE map2D_nws_ros2.h
  EXTRA_CONFIG WRAPPER=map2D_nws_ros2
  INTERNAL ON
)

yarp_add_plugin(yarp_map2D_nws_ros2)

target_sources(yarp_map2D_nws_ros2
  PRIVATE
    map2D_nws_ros2.cpp
    map2D_nws_ros2.h
)

target_link_libraries(yarp_map2D_nws_ros2
  PRIVATE
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_dev
    rclcpp::rclcpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
    nav_msgs::nav_msgs__rosidl_typesupport_cpp
    test_msgs::test_msgs__rosidl_typesupport_cpp
    visualization_msgs::visualization_msgs__rosidl_typesupport_cpp
    map2d_nws_ros2_msgs::map2d_nws_ros2_msgs__rosidl_typesupport_cpp
)

yarp_install(
  TARGETS yarp_map2D_nws_ros2
  EXPORT yarp-device-map2D_nws_ros2
  COMPONENT yarp-device-map2D_nws_ros2
  LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
  ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
  YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
)

set_property(TARGET yarp_map2D_nws_ros2 PROPERTY FOLDER "Plugins/Device")
