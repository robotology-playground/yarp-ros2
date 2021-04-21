# Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

#if(NOT YARP_COMPILE_DEVICE_PLUGINS)
#  return()
#endif()

add_library(Ros2ConversionUtils OBJECT)

target_sources(Ros2ConversionUtils PRIVATE Ros2ConversionUtils.cpp
                                              Ros2ConversionUtils.h
                                              ros2PixelCode.h
                                              ros2PixelCode.cpp)
                                              
target_include_directories(Ros2ConversionUtils PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(Ros2ConversionUtils PRIVATE YARP::YARP_os
                                                     YARP::YARP_sig
                                                     rclcpp::rclcpp
                                                     sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
                                                     std_msgs::std_msgs__rosidl_typesupport_c
                                                     YARP::YARP_dev)

set_property(TARGET Ros2ConversionUtils PROPERTY FOLDER "Libraries/Msgs")
