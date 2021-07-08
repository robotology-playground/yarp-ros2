# Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

yarp_prepare_plugin(subEx
  CATEGORY device
  TYPE SubEx
  INCLUDE subscriber_example.h
  INTERNAL ON
)

yarp_add_plugin(yarp_subEx)

target_sources(yarp_subEx
  PRIVATE
  subscriber_example.cpp
  subscriber_example.h
)

target_link_libraries(yarp_subEx
  PRIVATE
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_dev
    rclcpp::rclcpp
    std_msgs::std_msgs__rosidl_typesupport_c
)

yarp_install(
  TARGETS yarp_subEx
  EXPORT yarp-device-subEx
  COMPONENT yarp-device-subEx
  LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
  ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
  YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
)

set_property(TARGET yarp_subEx PROPERTY FOLDER "Plugins/Device")
