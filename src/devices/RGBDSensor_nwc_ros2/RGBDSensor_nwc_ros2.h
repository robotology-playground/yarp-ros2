/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_RGBDSENSOR_NWC_ROS2
#define YARP_RGBDSENSOR_NWC_ROS2

// c++ libraries
#include <iostream>
#include <cstring>
#include <mutex>

// yarp libraries
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/IVisualParams.h>
#include <yarp/os/Property.h>
#include <yarp/os/Thread.h>
#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/impl/FixedSizeBuffersManager.h>
#include <yarp/sig/IntrinsicParams.h>


// ros libraries
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>


/**
 *  @ingroup dev_impl_wrapper
 *
 * \section RGBDSensor_nwc_ros2_device_parameters Description of input parameters
 * This device is an nwc take a stream of data from ros2 and exposes to the user IRGBDSensor interface.
 * See its documentation for more details about the interface.
 *
 *   Parameters required by this device are:
 * | Parameter name         | SubParameter            | Type    | Units          | Default Value | Required                        | Description                                                                                         | Notes |
 * |:----------------------:|:-----------------------:|:-------:|:--------------:|:-------------:|:------------------------------: |:---------------------------------------------------------------------------------------------------:|:-----:|
 * | rgb_data_topic         |      -                  | string  |  -             |   -           |  no                             | ros rgb topic                                                                                       | must start with a leading '/' |
 * | rgb_info_topic         |      -                  | string  |  -             |               |  no                             | ros rgb camera info topic                                                                           | must start with a leading '/' |
 * | depth_data_topic       |      -                  | string  |  -             |   -           |  no                             | ros depth topic                                                                                     | must start with a leading '/' |
 * | depth_info_topic       |      -                  | string  |  -             |   -           |  no                             | ros depth camera info topic                                                                         | must start with a leading '/' |
 *
 * example of configuration file:
 *
 * Example of configuration parameters:
 *
 * \code{.unparsed}
 * device RGBDSensor_nwc_ros2
 * rgb_data_topic /<robotName>/camera/color/image_raw
 * rgb_info_topic /<robotName>/camera/color/camera_info
 * depth_data_topic /<robotName>/depth/color/image_rect_raw
 * depth_info_topic /<robotName>/depth/color/camera_info
 * \endcode
 */
/**
 * This class is an utility 
 */
class Ros2InitRGBDSensor_nwc_ros2
{
public:
    Ros2InitRGBDSensor_nwc_ros2();

    std::shared_ptr<rclcpp::Node> node;

    static Ros2InitRGBDSensor_nwc_ros2& get(); // do I need to make it static??
};

typedef yarp::sig::ImageOf<yarp::sig::PixelFloat> depthImage;
typedef yarp::sig::FlexImage flexImage;

/**
 * This class implements an nwc for ros2 for an rgbd sensor.
 */
class RGBDSensor_nwc_ros2:
        public yarp::dev::DeviceDriver,
        public yarp::os::Thread,
        public yarp::dev::IRGBDSensor
{
private:
	// mutex for writing or retrieving images
	std::mutex rgb_camera_info_mutex;
	std::mutex rgb_image_mutex;
	std::mutex depth_camera_info_mutex;
	std::mutex depth_image_mutex;

    // currentImages
    yarp::os::Stamp current_depth_stamp;
    depthImage current_depth_image;
    std::string depth_image_frame;
    yarp::sig::IntrinsicParams depth_params;
    double max_depth_width;
    double max_depth_height;

    yarp::os::Stamp current_rgb_stamp;
    flexImage current_rgb_image;
    std::string rgb_image_frame;
    yarp::sig::IntrinsicParams rgb_params;
    double max_rgb_width;
    double max_rgb_height;

    bool depth_image_valid = false;
    bool depth_stamp_valid = false;
    bool rgb_image_valid = false;
    bool rgb_stamp_valid = false;


	// ros2 variables for topics and subscriptions
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_subscription_rgb_camera_info;
    std::string                                                   m_topic_rgb_camera_info;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_subscription_rgb_image_raw;
    std::string                                              m_topic_rgb_image_raw;
    
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_subscription_depth_camera_info;
    std::string                                                   m_topic_depth_camera_info;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_subscription_depth_image_raw;
    std::string                                              m_topic_depth_image_raw;
    
    // yarp variables
    int 	m_verbose{2};

    //private functions
    void saveIntrinsics(sensor_msgs::msg::CameraInfo::SharedPtr msg, yarp::sig::IntrinsicParams& params);
    void depth_raw_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void depth_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void color_raw_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void color_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

public:
    RGBDSensor_nwc_ros2();
    RGBDSensor_nwc_ros2(const RGBDSensor_nwc_ros2&) = delete;
    RGBDSensor_nwc_ros2(RGBDSensor_nwc_ros2&&) noexcept = delete;
    RGBDSensor_nwc_ros2& operator=(const RGBDSensor_nwc_ros2&) = delete;
    RGBDSensor_nwc_ros2& operator=(RGBDSensor_nwc_ros2&&) noexcept = delete;
    ~RGBDSensor_nwc_ros2() override = default;

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // Thread
    void run() override;
    
    // IRGBDSensor
    int    getRgbHeight() override;
    int    getRgbWidth() override;
    bool   getRgbSupportedConfigurations(yarp::sig::VectorOf<yarp::dev::CameraConfig> &configurations) override;
    bool   getRgbResolution(int &width, int &height) override;
    bool   setRgbResolution(int width, int height) override;
    bool   getRgbFOV(double& horizontalFov, double& verticalFov) override;
    bool   setRgbFOV(double horizontalFov, double verticalFov) override;
    bool   getRgbMirroring(bool& mirror) override;
    bool   setRgbMirroring(bool mirror) override;

    bool   getRgbIntrinsicParam(yarp::os::Property& intrinsic) override;
    int    getDepthHeight() override;
    int    getDepthWidth() override;
    bool   setDepthResolution(int width, int height) override;
    bool   getDepthFOV(double& horizontalFov, double& verticalFov) override;
    bool   setDepthFOV(double horizontalFov, double verticalFov) override;
    bool   getDepthIntrinsicParam(yarp::os::Property& intrinsic) override;
    double getDepthAccuracy() override;
    bool   setDepthAccuracy(double accuracy) override;
    bool   getDepthClipPlanes(double& nearPlane, double& farPlane) override;
    bool   setDepthClipPlanes(double nearPlane, double farPlane) override;
    bool   getDepthMirroring(bool& mirror) override;
    bool   setDepthMirroring(bool mirror) override;

    bool   getExtrinsicParam(yarp::sig::Matrix &extrinsic) override;
    
    bool   getRgbImage(yarp::sig::FlexImage& rgb_image, yarp::os::Stamp* rgb_image_stamp = NULL) override;
    bool   getDepthImage(depthImage& depth_image, yarp::os::Stamp* depth_image_stamp = nullptr) override;
    bool   getImages(yarp::sig::FlexImage& rgb_image, depthImage& depth_image, yarp::os::Stamp* rgb_image_stamp=nullptr, yarp::os::Stamp* depth_image_stamp=nullptr) override;

    yarp::dev::IRGBDSensor::RGBDSensor_status     	getSensorStatus() override;
    std::string 			                        getLastErrorMsg(yarp::os::Stamp* timeStamp = NULL) override;
};

#endif // YARP_RGBDSENSOR_NWC_ROS2
