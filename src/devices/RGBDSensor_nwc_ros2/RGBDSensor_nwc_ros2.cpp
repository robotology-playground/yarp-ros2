/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "RGBDSensor_nwc_ros2.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include<Ros2ConversionUtils.h>

using namespace std::chrono_literals;
using namespace std::placeholders;

YARP_LOG_COMPONENT(RGBDSENSOR_NWC_ROS2, "yarp.ros2.RGBDSensor_nwc_ros2", yarp::os::Log::TraceType);


Ros2InitRGBDSensor_nwc_ros2::Ros2InitRGBDSensor_nwc_ros2()
{
    rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    node = std::make_shared<rclcpp::Node>("yarprobotinterface_node");
}

Ros2InitRGBDSensor_nwc_ros2& Ros2InitRGBDSensor_nwc_ros2::get()
{
    static Ros2InitRGBDSensor_nwc_ros2 instance;
    return instance;
}


RGBDSensor_nwc_ros2::RGBDSensor_nwc_ros2()
{
}

bool RGBDSensor_nwc_ros2::open(yarp::os::Searchable& config)
{
    m_topic_rgb_image_raw = "/camera/color/image_raw";
    m_topic_rgb_camera_info = "/camera/color/camera_info";
    m_topic_depth_image_raw = "/camera/depth/image_rect_raw";
    m_topic_depth_camera_info = "/camera/depth/camera_info";

    yCTrace(RGBDSENSOR_NWC_ROS2);
    if (config.check("rgb_data_topic"))   { m_topic_rgb_image_raw = config.find("rgb_data_topic").asString();}
    if (config.check("rgb_info_topic"))   { m_topic_rgb_camera_info = config.find("rgb_info_topic").asString(); }
    if (config.check("depth_data_topic")) { m_topic_depth_image_raw = config.find("depth_data_topic").asString(); }
    if (config.check("depth_info_topic")) { m_topic_depth_camera_info = config.find("depth_info_topic").asString(); }
    m_verbose = config.check("verbose");

    m_subscription_rgb_image_raw = Ros2InitRGBDSensor_nwc_ros2::get().node->create_subscription<sensor_msgs::msg::Image>(m_topic_rgb_image_raw, 10, std::bind(&RGBDSensor_nwc_ros2::color_raw_callback, this, _1));
    m_subscription_rgb_camera_info = Ros2InitRGBDSensor_nwc_ros2::get().node->create_subscription<sensor_msgs::msg::CameraInfo>(m_topic_rgb_camera_info, 10, std::bind(&RGBDSensor_nwc_ros2::color_info_callback, this, _1));
    m_subscription_depth_image_raw = Ros2InitRGBDSensor_nwc_ros2::get().node->create_subscription<sensor_msgs::msg::Image>(m_topic_depth_image_raw, 10, std::bind(&RGBDSensor_nwc_ros2::depth_raw_callback, this, _1));
    m_subscription_depth_camera_info = Ros2InitRGBDSensor_nwc_ros2::get().node->create_subscription<sensor_msgs::msg::CameraInfo>(m_topic_depth_camera_info, 10, std::bind(&RGBDSensor_nwc_ros2::depth_info_callback, this, _1));

    start();
    return true;
}

bool RGBDSensor_nwc_ros2::close()
{
    yCTrace(RGBDSENSOR_NWC_ROS2);
    yCInfo(RGBDSENSOR_NWC_ROS2, "shutting down");
    rclcpp::shutdown();
    return true;
}

void RGBDSensor_nwc_ros2::run()
{
    yCTrace(RGBDSENSOR_NWC_ROS2);
    yCInfo(RGBDSENSOR_NWC_ROS2, "starting");
    rclcpp::spin(Ros2InitRGBDSensor_nwc_ros2::get().node);
}

void RGBDSensor_nwc_ros2::depth_raw_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> depth_image_guard(depth_image_mutex);
    yarp::dev::Ros2ConversionUtils::convertDepthImageRos2ToYarpImageOf(msg,current_depth_image);
    depth_image_valid = true;
}

void RGBDSensor_nwc_ros2::depth_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    std::lock_guard<std::mutex> depth_camera_info_guard(depth_camera_info_mutex);
    yarp::dev::Ros2ConversionUtils::updateStamp(msg, depth_image_frame, current_depth_stamp);
    saveIntrinsics(msg, depth_params);
    max_depth_height = msg->height;
    max_depth_width = msg->width;
    yCInfo(RGBDSENSOR_NWC_ROS2) << "depth " << msg->width;
    depth_stamp_valid = true;
}

void RGBDSensor_nwc_ros2::color_raw_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> rgb_image_guard(rgb_image_mutex);
    yarp::dev::Ros2ConversionUtils::convertRGBImageRos2ToYarpFlexImage(msg, current_rgb_image);
    rgb_image_valid = true;
}

void RGBDSensor_nwc_ros2::color_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    std::lock_guard<std::mutex> rgb_camera_info_guard(rgb_camera_info_mutex);
    yarp::dev::Ros2ConversionUtils::updateStamp(msg, rgb_image_frame, current_rgb_stamp);
    saveIntrinsics(msg, rgb_params);
    max_rgb_height = msg->height;
    max_rgb_width = msg->width;
    yCInfo(RGBDSENSOR_NWC_ROS2) << "rgb " << msg->width;
    rgb_stamp_valid = true;
}

// TODO FIXME eable distortion inverse brown conrady or check distortion in realsense
void RGBDSensor_nwc_ros2::saveIntrinsics(sensor_msgs::msg::CameraInfo::SharedPtr msg, yarp::sig::IntrinsicParams& params)
{
    params.focalLengthX = msg->k[0];
    params.focalLengthY = msg->k[4];
    params.principalPointX = msg->k[2];
    params.principalPointY = msg->k[5];
    // distortion model
    if (msg->distortion_model=="plumb_bob")
    {
        if (params.distortionModel.k1 == 0 &&
            params.distortionModel.k2 == 0 && 
            params.distortionModel.k3 == 0 && 
            params.distortionModel.t1 == 0 &&
            params.distortionModel.t2 == 0) 
        {
            params.distortionModel.type = yarp::sig::YarpDistortion::YARP_DISTORTION_NONE;
        }
        params.distortionModel.type = yarp::sig::YarpDistortion::YARP_PLUM_BOB;
        params.distortionModel.k1 = msg->d[0];
        params.distortionModel.k2 = msg->d[1];
        params.distortionModel.t1 = msg->d[2];
        params.distortionModel.t2 = msg->d[3];
        params.distortionModel.k3 = msg->d[4];
    }
    else
    {
        yCError(RGBDSENSOR_NWC_ROS2) << "Unsupported distortion model";
        params.distortionModel.type = yarp::sig::YarpDistortion::YARP_UNSUPPORTED;
    }
}



int RGBDSensor_nwc_ros2::getRgbHeight()
{
    if (!rgb_image_valid) return 0;
    return current_rgb_image.height();
}

int RGBDSensor_nwc_ros2::getRgbWidth()
{
    if (!rgb_image_valid) return 0;
    return current_rgb_image.width();
}

bool RGBDSensor_nwc_ros2::getRgbSupportedConfigurations(yarp::sig::VectorOf<yarp::dev::CameraConfig> &configurations)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "getRgbSupportedConfigurations not implemented yet";
    return false;
}

bool RGBDSensor_nwc_ros2::getRgbResolution(int &width, int &height)
{
    if (!rgb_image_valid)
    {
        width=0;
        height=0;
        return  false;
    }
    width  = current_rgb_image.width();
    height = current_rgb_image.height();
    return true;
}

bool RGBDSensor_nwc_ros2::setDepthResolution(int width, int height)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "setDepthResolution not supported";
    return false;
}

bool RGBDSensor_nwc_ros2::setRgbResolution(int width, int height)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "setRgbResolution not supported";
    return false;
}

bool RGBDSensor_nwc_ros2::setRgbFOV(double horizontalFov, double verticalFov)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "setRgbFOV not supported";
    return false;
}

bool RGBDSensor_nwc_ros2::setDepthFOV(double horizontalFov, double verticalFov)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "setDepthFOV not supported";
    return false;
}

bool RGBDSensor_nwc_ros2::setDepthAccuracy(double accuracy)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "setDepthAccuracy not supported";
    return false;
}

// not sure that it is correct, maybe I should save the full image dimension
bool RGBDSensor_nwc_ros2::getRgbFOV(double &horizontalFov, double &verticalFov)
{
    if (!rgb_image_valid && !rgb_stamp_valid)
    {
        horizontalFov=0;
        verticalFov=0;
        return  false;
        yCError(RGBDSENSOR_NWC_ROS2) << "get rgb IntrinsicParam missing information";
    }
    horizontalFov = 2 * atan(max_rgb_width / (2 * rgb_params.focalLengthX)) * 180.0 / M_PI;
    verticalFov = 2 * atan(max_rgb_height / (2 * rgb_params.focalLengthY)) * 180.0 / M_PI;
    return true;
}


bool RGBDSensor_nwc_ros2::getRgbMirroring(bool& mirror)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "Mirroring not supported";
    return false;
}

bool RGBDSensor_nwc_ros2::setRgbMirroring(bool mirror)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "Mirroring not supported";
    return false;
}

bool RGBDSensor_nwc_ros2::getRgbIntrinsicParam(yarp::os::Property& intrinsic)
{
    if (rgb_stamp_valid)
    {
        intrinsic.clear();
        rgb_params.toProperty(intrinsic);
        return true;
    }
    return false;
}

int  RGBDSensor_nwc_ros2::getDepthHeight()
{
    if (!depth_image_valid) return 0;
    return current_depth_image.height();
}

int  RGBDSensor_nwc_ros2::getDepthWidth()
{
    if (!depth_image_valid) return 0;
    return current_depth_image.width();
}

bool RGBDSensor_nwc_ros2::getDepthFOV(double& horizontalFov, double& verticalFov)
{
    if (!depth_image_valid && !depth_stamp_valid)
    {
        horizontalFov=0;
        verticalFov=0;
        return  false;
        yCError(RGBDSENSOR_NWC_ROS2) << "get depth IntrinsicParam missing information";
    }
    horizontalFov = 2 * atan(max_rgb_width / (2 * depth_params.focalLengthX)) * 180.0 / M_PI;
    verticalFov = 2 * atan(max_rgb_height / (2 * depth_params.focalLengthY)) * 180.0 / M_PI;
    return true;
}

bool RGBDSensor_nwc_ros2::getDepthIntrinsicParam(yarp::os::Property& intrinsic)
{
    if(depth_stamp_valid) 
    {
        intrinsic.clear();
        depth_params.toProperty(intrinsic);
        return true;
    }
    return false;
}

double RGBDSensor_nwc_ros2::getDepthAccuracy()
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "getDepthAccuracy not supported";
    return 0;
}

bool RGBDSensor_nwc_ros2::getDepthClipPlanes(double& nearPlane, double& farPlane)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "getDepthClipPlanes not supported";
    return false;
}

bool RGBDSensor_nwc_ros2::setDepthClipPlanes(double nearPlane, double farPlane)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "setDepthClipPlanes not supported";
    return false;
}

bool RGBDSensor_nwc_ros2::getDepthMirroring(bool& mirror)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "getDepthMirroring not supported";
    return false;
}

bool RGBDSensor_nwc_ros2::setDepthMirroring(bool mirror)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "setDepthMirroring not supported";
    return false;
}

bool RGBDSensor_nwc_ros2::getExtrinsicParam(yarp::sig::Matrix& extrinsic)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "getExtrinsicParam not supported";
    return  false;
}


/*
 * TODO FIXME to update with header instead of stamp
 */
bool RGBDSensor_nwc_ros2::getRgbImage(yarp::sig::FlexImage& rgb_image, yarp::os::Stamp* rgb_image_stamp)
{
    std::lock_guard<std::mutex> guard_rgb_image(rgb_image_mutex);
    std::lock_guard<std::mutex> guard_rgb_camera_info(rgb_camera_info_mutex);
    bool rgb_ok = false;
    if (rgb_image_valid)
    {
        if (rgb_stamp_valid)
        {
            if (rgb_image_stamp == nullptr)
            {
                yarp::os::Stamp stamp(current_rgb_stamp.getCount(),current_rgb_stamp.getTime());
                *rgb_image_stamp = stamp;
            }
            yarp::dev::Ros2ConversionUtils::deepCopyFlexImage(current_rgb_image, rgb_image);
            rgb_ok = true;
        }
        else
        {
            yCError(RGBDSENSOR_NWC_ROS2, "missing rgb camera info");
        }
    }
    else
    {
        yCError(RGBDSENSOR_NWC_ROS2, "missing rgb image");
    }
    return rgb_ok;
}

/*
 * TODO FIXME to update with header instead of stamp
 */
bool RGBDSensor_nwc_ros2::getDepthImage(depthImage& depth_image, yarp::os::Stamp* depth_image_stamp)
{
    std::lock_guard<std::mutex> guard_depth_image(depth_image_mutex);
    std::lock_guard<std::mutex> guard_depth_camera_info(depth_camera_info_mutex);
    bool depth_ok =false;

    if (depth_image_valid)
    {
        if (depth_stamp_valid)
        {
            if (depth_image_stamp == nullptr)
            {
                yarp::os::Stamp stamp(current_depth_stamp.getCount(),current_depth_stamp.getTime());
                *depth_image_stamp = stamp;
            }
            yarp::dev::Ros2ConversionUtils::deepCopyImageOf(current_depth_image, depth_image);
            depth_ok = true;
        }
        else
        {
            yCError(RGBDSENSOR_NWC_ROS2, "missing depth camera info");
        }
    }
    else
    {
        yCError(RGBDSENSOR_NWC_ROS2, "missing depth image");
    }
    return depth_ok;
}
/*
 * TODO FIXME to update with header instead of stamp
 */
bool RGBDSensor_nwc_ros2::getImages(yarp::sig::FlexImage& rgb_image, depthImage& depth_image, yarp::os::Stamp* rgb_image_stamp, yarp::os::Stamp* depth_image_stamp)
{
    bool rgb_ok, depth_ok;
    rgb_ok = getRgbImage(rgb_image, rgb_image_stamp);
    depth_ok = getDepthImage(depth_image, depth_image_stamp);
    return (rgb_ok && depth_ok);
}


RGBDSensor_nwc_ros2::RGBDSensor_status RGBDSensor_nwc_ros2::getSensorStatus()
{
    return RGBD_SENSOR_OK_IN_USE;
}


std::string RGBDSensor_nwc_ros2::getLastErrorMsg(yarp::os::Stamp* timeStamp)
{
    yCError(RGBDSENSOR_NWC_ROS2) << "get last error not yet implemented";
    return "get last error not yet implemented";
}