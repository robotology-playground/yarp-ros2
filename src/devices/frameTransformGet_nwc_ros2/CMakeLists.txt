# Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

yarp_prepare_plugin(frameTransformGet_nwc_ros2
  CATEGORY device
  TYPE FrameTransformGet_nwc_ros2
  INCLUDE frameTransformGet_nwc_ros2.h
  INTERNAL ON
)

yarp_add_plugin(yarp_frameTransformGet_nwc_ros2)

target_sources(yarp_frameTransformGet_nwc_ros2
  PRIVATE
    frameTransformGet_nwc_ros2.cpp
    frameTransformGet_nwc_ros2.h
)

target_sources(yarp_frameTransformGet_nwc_ros2 PRIVATE $<TARGET_OBJECTS:FrameTransformUtils>)
target_include_directories(yarp_frameTransformGet_nwc_ros2 PRIVATE $<TARGET_PROPERTY:FrameTransformUtils,INTERFACE_INCLUDE_DIRECTORIES>)

target_link_libraries(yarp_frameTransformGet_nwc_ros2
  PRIVATE
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_dev
    rclcpp::rclcpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    tf2_msgs::tf2_msgs__rosidl_typesupport_cpp
)

yarp_install(
  TARGETS yarp_frameTransformGet_nwc_ros2
  EXPORT yarp-device-rframeTransformGet_nwc_ros2
  COMPONENT yarp-device-frameTransformGet_nwc_ros2
  LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
  ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
  YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
)

set_property(TARGET yarp_frameTransformGet_nwc_ros2 PROPERTY FOLDER "Plugins/Device")
