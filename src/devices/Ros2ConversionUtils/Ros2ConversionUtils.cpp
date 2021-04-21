/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <cmath>
#include <algorithm>
#include <iomanip>
#include <cstdint>

#include <yarp/os/LogComponent.h>
#include <yarp/os/Value.h>
#include <yarp/sig/ImageUtils.h>

#include "Ros2ConversionUtils.h"

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::dev::Ros2ConversionUtils;
using namespace std;

namespace {
YARP_LOG_COMPONENT(ROS2_CONVERSION_UTILS, "yarp.device.Ros2ConversionUtils");
}


void yarp::dev::Ros2ConversionUtils::convertTimeStampRos2ToYarp(const std_msgs::msg::Header& ros_header, yarp::os::Stamp& yarp_stamp)
{
    yarp_stamp.update(ros_header.stamp.sec + (ros_header.stamp.nanosec / ONE_MILLION));
}


void yarp::dev::Ros2ConversionUtils::convertTimeStampYarpToRos2(const yarp::os::Stamp& yarp_stamp, std_msgs::msg::Header& ros_header)
{
    ros_header.stamp.sec = int(yarp_stamp.getTime());
    ros_header.stamp.nanosec = int(ONE_MILLION * (yarp_stamp.getTime() - int(yarp_stamp.getTime())));
}

void yarp::dev::Ros2ConversionUtils::convertRGBImageRos2ToYarpFlexImage(sensor_msgs::msg::Image::SharedPtr ros_image_src,
                                                                        yarp::sig::FlexImage& dest)
{
    int yarp_pixcode = yarp::dev::ROS2PixelCode::Ros2ToYarpPixelCode(ros_image_src->encoding);
    if (yarp_pixcode == VOCAB_PIXEL_RGB ||
        yarp_pixcode == VOCAB_PIXEL_BGR)
    {
        dest.setQuantum(0);
        dest.setPixelCode(yarp_pixcode);
        dest.setPixelSize(ros_image_src->step/ros_image_src->width); // this I think is rowsize
        dest.resize(ros_image_src->width, ros_image_src->height);
        size_t c = 0;
        for (auto it = ros_image_src->data.begin(); it != ros_image_src->data.end(); it++)
        {
            dest.getRawImage()[c++]=*it;
        }
    }
    else
    {
        yCError(ROS2_CONVERSION_UTILS) << "Unsupported rgb format:" << ros_image_src->encoding;
    }

}


void yarp::dev::Ros2ConversionUtils::convertDepthImageRos2ToYarpImageOf(sensor_msgs::msg::Image::SharedPtr ros_image_src,
                                                                        yarp::sig::ImageOf<yarp::sig::PixelFloat>& dest)
{
    if (ros_image_src->encoding == TYPE_16UC1)
    {
        dest.resize(ros_image_src->width, ros_image_src->height);
        size_t c = 0;
        uint16_t* p = (uint16_t*)(ros_image_src->data.data());
        uint16_t* siz = (uint16_t*)(ros_image_src->data.data()) + (ros_image_src->data.size() / sizeof(uint16_t));
        int count = 0;
        for (; p < siz; p++)
        {
            float value = float(*p) / 1000.0;
            ((float*)(dest.getRawImage()))[c++] = value;
            count++;
        }
    }
    else if (ros_image_src->encoding == TYPE_32FC1)
    {
        dest.resize(ros_image_src->width, ros_image_src->height);
        size_t c = 0;
        for (auto it = ros_image_src->data.begin(); it != ros_image_src->data.end(); it++)
        {
            dest.getRawImage()[c++] = *it;
        }
    }
    else
    {
        yCError(ROS2_CONVERSION_UTILS) << "Unsupported depth format:" << ros_image_src->encoding;
    }
}


void yarp::dev::Ros2ConversionUtils::updateStamp(   sensor_msgs::msg::CameraInfo::SharedPtr ros_camera_info_src, 
                                                    std::string& frame_id_dest,
                                                    yarp::os::Stamp& yarp_stamp)
{
    frame_id_dest = ros_camera_info_src->header.frame_id;
    //how to do with seq??
    convertTimeStampRos2ToYarp(ros_camera_info_src->header, yarp_stamp);
}

void yarp::dev::Ros2ConversionUtils::deepCopyFlexImage(const yarp::sig::FlexImage& src, yarp::sig::FlexImage& dest)
{
    dest.setPixelCode(src.getPixelCode());
    dest.setQuantum(src.getQuantum());
    dest.resize(src.width(), src.height());
    for (size_t it = 0; it < src.getRawImageSize(); it++)
    {
        dest.getRawImage()[it] = src.getRawImage()[it];
    }
}

void yarp::dev::Ros2ConversionUtils::deepCopyImageOf(const DepthImage& src, DepthImage& dest)
{
    dest.setQuantum(src.getQuantum());
    dest.resize(src.width(), src.height());
    for (size_t it = 0; it < src.getRawImageSize(); it++)
    {
        dest.getRawImage()[it] = src.getRawImage()[it];
    }
}

