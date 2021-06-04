# Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

add_library(FrameTransformUtils OBJECT)

target_sources(FrameTransformUtils PRIVATE frameTransformContainer.cpp
                                           frameTransformContainer.h)

target_include_directories(FrameTransformUtils PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(FrameTransformUtils PRIVATE YARP::YARP_os
                                                  YARP::YARP_sig
                                                  YARP::YARP_dev)
