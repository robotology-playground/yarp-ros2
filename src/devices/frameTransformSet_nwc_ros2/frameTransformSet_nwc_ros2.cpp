/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "frameTransformSet_nwc_ros2.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

using namespace std;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

namespace {
YARP_LOG_COMPONENT(FRAMETRANSFORMSETNWCROS2, "yarp.device.frameTransformSet_nwc_ros2")
}

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

//------------------------------------------------------------------------------------------------------------------------------

FrameTransformSet_nwc_ros2::FrameTransformSet_nwc_ros2(double tperiod) :
PeriodicThread(tperiod),
m_period(tperiod)
{
}

bool FrameTransformSet_nwc_ros2::open(yarp::os::Searchable& config)
{
    if (!yarp::os::NetworkBase::checkNetwork()) {
        yCError(FRAMETRANSFORMSETNWCROS2,"Error! YARP Network is not initialized");
        return false;
    }

    bool okGeneral = config.check("GENERAL");
    if(okGeneral)
    {
        yarp::os::Searchable& general_config = config.findGroup("GENERAL");
        if (general_config.check("period"))
        {
            m_period = general_config.find("period").asFloat64();
            setPeriod(m_period);
        }
        if (general_config.check("refresh_interval"))  {m_refreshInterval = general_config.find("refresh_interval").asFloat64();}
        if (general_config.check("asynch_pub"))        {m_asynchPub = general_config.find("asynch_pub").asInt16();}
    }

    m_ftContainer.m_timeout = m_refreshInterval;

    //ROS2 configuration
    if (config.check("ROS2"))
    {
        yCInfo(FRAMETRANSFORMSETNWCROS2, "Configuring ROS2 params");
        Bottle ROS2_config = config.findGroup("ROS2");
        if(ROS2_config.check("ft_topic")) m_ftTopic = ROS2_config.find("ft_topic").asString();
        if(ROS2_config.check("ft_topic_static")) m_ftTopicStatic = ROS2_config.find("ft_topic_static").asString();
    }
    else
    {
        //no ROS2 options
        yCWarning(FRAMETRANSFORMSETNWCROS2) << "ROS2 Group not configured";
    }

    m_publisherFtTimed = Ros2Init::get().node->create_publisher<tf2_msgs::msg::TFMessage>(m_ftTopic, 10);
    m_publisherFtStatic = Ros2Init::get().node->create_publisher<tf2_msgs::msg::TFMessage>(m_ftTopicStatic, 10);

    start();

    return true;
}

bool FrameTransformSet_nwc_ros2::close()
{
    yCTrace(FRAMETRANSFORMSETNWCROS2, "Close");
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    return true;
}

void FrameTransformSet_nwc_ros2::run()
{
    std::lock_guard <std::mutex> lg(m_trf_mutex);
    if (!publishFrameTransforms())
    {
        yCError(FRAMETRANSFORMSETNWCROS2,"Error while publishing transforms");
    }
    return;
}

bool FrameTransformSet_nwc_ros2::setTransforms(const std::vector<yarp::math::FrameTransform>& transforms)
{
    std::lock_guard<std::mutex> lock(m_trf_mutex);
    if(!m_ftContainer.setTransforms(transforms))
    {
        yCError(FRAMETRANSFORMSETNWCROS2,"Unable to set transforms");
        return false;
    }
    if(m_asynchPub)
    {
        if (!publishFrameTransforms())
        {
            yCError(FRAMETRANSFORMSETNWCROS2,"Error while publishing transforms");
            return false;
        }
    }
    return true;
}

bool FrameTransformSet_nwc_ros2::setTransform(const yarp::math::FrameTransform& t)
{
    std::lock_guard<std::mutex> lock(m_trf_mutex);
    if(!m_ftContainer.setTransform(t))
    {
        yCError(FRAMETRANSFORMSETNWCROS2,"Unable to set transform");
        return false;
    }
    if(m_asynchPub)
    {
        if (!publishFrameTransforms())
        {
            yCError(FRAMETRANSFORMSETNWCROS2,"Error while publishing transforms");
            return false;
        }
    }
    return true;
}

void FrameTransformSet_nwc_ros2::ros2TimeFromYarp(double yarpTime, builtin_interfaces::msg::Time& ros2Time)
{
    uint64_t time;
    uint64_t sec_part;
    uint64_t nsec_part;
    time = (uint64_t)(yarpTime * 1000000000UL);
    sec_part = (time / 1000000000UL);
    nsec_part = time - sec_part*1000000000UL;
    ros2Time.sec = sec_part;
    ros2Time.nanosec = (uint32_t)nsec_part;
}

void FrameTransformSet_nwc_ros2::yarpTransformToROS2Transform(const yarp::math::FrameTransform &input, geometry_msgs::msg::TransformStamped& output)
{
    builtin_interfaces::msg::Time tempStamp;
    std_msgs::msg::Header tempHeader;
    geometry_msgs::msg::Transform tempTf;
    geometry_msgs::msg::Vector3 tempTransl;
    geometry_msgs::msg::Quaternion tempRotation;
    
    tempTransl.x = input.translation.tX;
    tempTransl.y = input.translation.tY;
    tempTransl.z = input.translation.tY;

    tempRotation.w = input.rotation.w();
    tempRotation.x = input.rotation.x();
    tempRotation.y = input.rotation.y();
    tempRotation.z = input.rotation.z();

    tempTf.rotation = tempRotation;
    tempTf.translation = tempTransl;

    ros2TimeFromYarp(input.isStatic ? yarp::os::Time::now() : input.timestamp,tempStamp);
    tempHeader.stamp = tempStamp;
    tempHeader.frame_id = input.src_frame_id;
    
    output.header = tempHeader;
    output.child_frame_id = input.dst_frame_id;
    output.transform = tempTf;
}

bool FrameTransformSet_nwc_ros2::publishFrameTransforms()
{
    if(!m_ftContainer.checkAndRemoveExpired())
    {
        yCError(FRAMETRANSFORMSETNWCROS2,"Unable to remove expired transforms");
        return false;
    }
    std::vector<yarp::math::FrameTransform> fromGet;
    m_ftContainer.getTransforms(fromGet);
    tf2_msgs::msg::TFMessage toPublish_timed;
    tf2_msgs::msg::TFMessage toPublish_static;
    std::vector<geometry_msgs::msg::TransformStamped> content_timed;
    std::vector<geometry_msgs::msg::TransformStamped> content_static;
    for(auto &ft : fromGet)
    {
        geometry_msgs::msg::TransformStamped tempTfs;
        yarpTransformToROS2Transform(ft,tempTfs);
        if(ft.isStatic) content_timed.push_back(tempTfs);
        if(ft.isStatic) content_static.push_back(tempTfs);
    }

    toPublish_timed.transforms = content_timed;
    m_publisherFtTimed->publish(toPublish_timed);
    toPublish_static.transforms = content_static;
    m_publisherFtStatic->publish(toPublish_static);

    return true;
}
