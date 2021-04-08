/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "RGBDToPointCloudSensor_nws_ros2.h"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <rclcpp/time.hpp>
#include <string>
#include <vector>
#include <yarp/sig/PointCloudUtils.h>
//#include <builtin_interfaces/Time>
#include <sensor_msgs/image_encodings.hpp>

using namespace std::chrono_literals;
using namespace std;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::os;
using namespace RGBDToPointCloudRos2Impl;
YARP_LOG_COMPONENT(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2, "yarp.ros2.RGBDToPointCloudSensor_nws_ros2", yarp::os::Log::TraceType);


Ros2Init::Ros2Init()
{
    rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    node = std::make_shared<rclcpp::Node>("yarprobotinterface_node");
}

Ros2Init& Ros2Init::get()
{
    static Ros2Init instance;
    return instance;
}


RGBDToPointCloudSensor_nws_ros2::RGBDToPointCloudSensor_nws_ros2() :
        yarp::os::PeriodicThread(DEFAULT_THREAD_PERIOD)
{
}


// DeviceDriver
bool RGBDToPointCloudSensor_nws_ros2::open(yarp::os::Searchable &config)
{
    yCDebug(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2) << "Parameters are: " << config.toString();

    if(!fromConfig(config)) {
        yCError(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2) << "Failed to open, check previous log for error messages.";
        return false;
    }

    if(!initialize_ROS2(config)) {
        yCError(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2) << "Error initializing ROS topic";
        return false;
    }

    // check if we need to create subdevice or if they are
    // passed later on through attachAll()
    if (isSubdeviceOwned && !openAndAttachSubDevice(config)) {
        yCError(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2, "Error while opening subdevice");
        return false;
    }

    return true;
}


bool RGBDToPointCloudSensor_nws_ros2::fromConfig(yarp::os::Searchable &config)
{
    if (!config.check("period", "refresh period of the broadcasted values in s")) {
        yCDebug(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2) << "Using default 'period' parameter of " << DEFAULT_THREAD_PERIOD << "s";
    } else {
        setPeriod(config.find("period").asFloat64());
    }

    //check if param exist and assign it to corresponding variable.. if it doesn't, initialize the variable with default value.
    std::vector<param<std::string>> rosStringParam;

    rosStringParam.emplace_back(nodeName,       nodeName_param          );
    rosStringParam.emplace_back(rosFrameId,     frameId_param           );
    rosStringParam.emplace_back(pointCloudTopicName, pointCloudTopicName_param    );

    for (auto &prm : rosStringParam) {
        if (!config.check(prm.parname)) {
            yCError(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2) << "Missing " << prm.parname << "check your configuration file";
            return false;
        }
        *(prm.var) = config.find(prm.parname).asString();
    }

    if (config.check("forceInfoSync"))
    {
        forceInfoSync = config.find("forceInfoSync").asBool();
    }

    if(config.check("subdevice")) {
        isSubdeviceOwned=true;
    } else {
        isSubdeviceOwned=false;
    }

    return true;
}


bool RGBDToPointCloudSensor_nws_ros2::initialize_ROS2(yarp::os::Searchable &params)
{
    rosPublisher_pointCloud2 = Ros2Init::get().node->create_publisher<sensor_msgs::msg::PointCloud2>(pointCloudTopicName, 10);
    return true;
}


bool RGBDToPointCloudSensor_nws_ros2::close()
{
    yCTrace(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2, "Close");
    detachAll();

    // close subdevice if it was created inside the open (--subdevice option)
    if(isSubdeviceOwned)
    {
        if(subDeviceOwned)
        {
            delete subDeviceOwned;
            subDeviceOwned=nullptr;
        }
        sensor_p = nullptr;
        fgCtrl = nullptr;
        isSubdeviceOwned = false;
    }

    return true;
}


// PeriodicThread
void RGBDToPointCloudSensor_nws_ros2::run()
{
    if (sensor_p!=nullptr) {
        static int i = 0;
        switch (sensor_p->getSensorStatus()) {
            case(yarp::dev::IRGBDSensor::RGBD_SENSOR_OK_IN_USE) :
            if (!writeData()) {
                yCError(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2, "Image not captured.. check hardware configuration");
            }
            i = 0;
            break;
        case(yarp::dev::IRGBDSensor::RGBD_SENSOR_NOT_READY):
            if(i < 1000) {
                if((i % 30) == 0) {
                    yCInfo(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2) << "Device not ready, waiting...";
                    }
                } else {
                    yCWarning(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2) << "Device is taking too long to start..";
                }
                i++;
            break;
        default:
            yCError(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2, "Sensor returned error");
        }
    } else {
        yCError(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2, "Sensor interface is not valid");
    }
}


/*
 * IWrapper and IMultipleWrapper interfaces
 */
bool RGBDToPointCloudSensor_nws_ros2::attachAll(const yarp::dev::PolyDriverList &device2attach)
{
    // First implementation only accepts devices with both the interfaces Framegrabber and IDepthSensor,
    // on a second version maybe two different devices could be accepted, one for each interface.
    // Yet to be defined which and how parameters shall be used in this case ... using the name of the
    // interface to view could be a good initial guess.
    if (device2attach.size() != 1)
    {
        yCError(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2, "Cannot attach more than one device");
        return false;
    }

    yarp::dev::PolyDriver * Idevice2attach = device2attach[0]->poly;

    if (!Idevice2attach->isValid()) {
        yCError(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2) << "Device " << device2attach[0]->key << " to attach to is not valid ... cannot proceed";
        return false;
    }

    Idevice2attach->view(sensor_p);
    Idevice2attach->view(fgCtrl);
    if(!attach(sensor_p)) {
        return false;
    }

    return start();
}

bool RGBDToPointCloudSensor_nws_ros2::detachAll()
{
    if (isRunning()) {
        stop();
    }

    //check if we already instantiated a subdevice previously
    if (isSubdeviceOwned) {
        return false;
    }

    sensor_p = nullptr;
    return true;
}

bool RGBDToPointCloudSensor_nws_ros2::attach(yarp::dev::IRGBDSensor *s)
{
    if(s == nullptr) {
        yCError(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2) << "Attached device has no valid IRGBDSensor interface.";
        return false;
    }
    sensor_p = s;

    return start();
}

bool RGBDToPointCloudSensor_nws_ros2::attach(yarp::dev::PolyDriver* poly)
{
    if(poly) {
        poly->view(sensor_p);
        poly->view(fgCtrl);
    }

    if(sensor_p == nullptr) {
        yCError(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2) << "Attached device has no valid IRGBDSensor interface.";
        return false;
    }

    return start();
}

bool RGBDToPointCloudSensor_nws_ros2::detach()
{
    sensor_p = nullptr;
    return true;
}


bool RGBDToPointCloudSensor_nws_ros2::openAndAttachSubDevice(yarp::os::Searchable& prop)
{
    yarp::os::Property p;
    subDeviceOwned = new yarp::dev::PolyDriver;
    p.fromString(prop.toString());

    p.setMonitor(prop.getMonitor(), "subdevice"); // pass on any monitoring
    p.unput("device");
    p.put("device",prop.find("subdevice").asString());  // subdevice was already checked before

    // if errors occurred during open, quit here.
    yCDebug(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2, "Opening IRGBDSensor subdevice");
    subDeviceOwned->open(p);

    if (!subDeviceOwned->isValid())
    {
        yCError(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2, "Opening IRGBDSensor subdevice... FAILED");
        return false;
    }
    isSubdeviceOwned = true;
    if(!attach(subDeviceOwned)) {
        return false;
    }

    return true;
}


bool RGBDToPointCloudSensor_nws_ros2::writeData()
{
    yarp::sig::FlexImage colorImage;
    yarp::sig::ImageOf<yarp::sig::PixelFloat> depthImage;
    yarp::os::Stamp colorStamp;
    yarp::os::Stamp depthStamp;

    if (!sensor_p->getImages(colorImage, depthImage, &colorStamp, &depthStamp)) {
        return false;
    }

    static Stamp oldColorStamp = Stamp(0, 0);
    static Stamp oldDepthStamp = Stamp(0, 0);
    yarp::os::Property propIntrinsic;
    bool rgb_data_ok = true;
    bool depth_data_ok = true;
    bool intrinsic_ok = false;

    if (((colorStamp.getTime() - oldColorStamp.getTime()) > 0) == false) {
        rgb_data_ok = false;
    } else {
        oldColorStamp = colorStamp;
    }

    if (((depthStamp.getTime() - oldDepthStamp.getTime()) > 0) == false) {
        depth_data_ok = false;
    } else {
        oldDepthStamp = depthStamp;
    }
    intrinsic_ok = sensor_p->getRgbIntrinsicParam(propIntrinsic);


    // TBD: We should check here somehow if the timestamp was correctly updated and, if not, update it ourselves.
    if (rgb_data_ok) {
        if (depth_data_ok) {
            if (intrinsic_ok) {
                yarp::sig::IntrinsicParams intrinsics(propIntrinsic);
                yarp::sig::ImageOf<yarp::sig::PixelRgb> colorImagePixelRGB;
                colorImagePixelRGB.setExternal(colorImage.getRawImage(), colorImage.width(), colorImage.height());
                // create point cloud in yarp format
                yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> yarpCloud = yarp::sig::utils::depthRgbToPC<yarp::sig::DataXYZRGBA, yarp::sig::PixelRgb>(depthImage, colorImagePixelRGB, intrinsics);
                sensor_msgs::msg::PointCloud2 pc2Ros;

                // filling ros header
                pc2Ros.header.frame_id = rosFrameId;

                //pc2Ros.header.stamp.sec = depthStamp.;
                pc2Ros.header.stamp.sec = int(depthStamp.getTime());
                pc2Ros.header.stamp.nanosec = int(1000000 * (depthStamp.getTime() - int(depthStamp.getTime())));

                // filling ros point field
                pc2Ros.fields.push_back(sensor_msgs::msg::PointField());
                pc2Ros.fields.push_back(sensor_msgs::msg::PointField());
                pc2Ros.fields.push_back(sensor_msgs::msg::PointField());
                pc2Ros.fields.push_back(sensor_msgs::msg::PointField());
                pc2Ros.fields[0].name = "x";
                pc2Ros.fields[0].offset = 0;
                pc2Ros.fields[0].datatype = 7;
                pc2Ros.fields[0].count = 1;
                pc2Ros.fields[1].name = "y";
                pc2Ros.fields[1].offset = 4;
                pc2Ros.fields[1].datatype = 7;
                pc2Ros.fields[1].count = 1;
                pc2Ros.fields[2].name = "z";
                pc2Ros.fields[2].offset = 8;
                pc2Ros.fields[2].datatype = 7;
                pc2Ros.fields[2].count = 1;
                pc2Ros.fields[3].name = "rgb";
                pc2Ros.fields[3].offset = 16;
                pc2Ros.fields[3].datatype = 7;
                pc2Ros.fields[3].count = 1;

                std::vector<unsigned char> vec(yarpCloud.getRawData(), yarpCloud.getRawData() + yarpCloud.dataSizeBytes());
                pc2Ros.data = vec;
                pc2Ros.width = yarpCloud.width() * yarpCloud.height();
                pc2Ros.height = 1;
                pc2Ros.is_dense = yarpCloud.isDense();

                pc2Ros.point_step = sizeof(yarp::sig::DataXYZRGBA);
                pc2Ros.row_step = static_cast<std::uint32_t>(sizeof(yarp::sig::DataXYZRGBA) * pc2Ros.width);
                rosPublisher_pointCloud2->publish(pc2Ros);
            }
        }
    }

    nodeSeq++;

    return true;
}
