# Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

yarp_prepare_plugin(RGBDToPointCloudSensor_nws_ros2
  CATEGORY device
  TYPE RGBDToPointCloudSensor_nws_ros2
  INCLUDE RGBDToPointCloudSensor_nws_ros2.h
  EXTRA_CONFIG WRAPPER=RGBDToPointCloudSensor_nws_ros2
  INTERNAL ON
)

yarp_add_plugin(yarp_RGBDToPointCloudSensor_nws_ros2)

target_sources(yarp_RGBDToPointCloudSensor_nws_ros2
  PRIVATE
    RGBDToPointCloudSensor_nws_ros2.cpp
    RGBDToPointCloudSensor_nws_ros2.h
)

target_link_libraries(yarp_RGBDToPointCloudSensor_nws_ros2
  PRIVATE
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_dev
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
)

yarp_install(
  TARGETS yarp_RGBDToPointCloudSensor_nws_ros2
  EXPORT yarp-device-RGBDToPointCloudSensor_nws_ros2
  COMPONENT yarp-device-RGBDToPointCloudSensor_nws_ros2
  LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
  ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
  YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
)

set_property(TARGET yarp_RGBDToPointCloudSensor_nws_ros2 PROPERTY FOLDER "Plugins/Device")
