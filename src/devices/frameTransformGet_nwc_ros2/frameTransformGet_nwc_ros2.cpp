/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "frameTransformGet_nwc_ros2.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

using namespace std;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace std::placeholders;

namespace {
YARP_LOG_COMPONENT(FRAMETRANSFORGETNWCROS2, "yarp.device.frameTransformGet_nwc_ros2")
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

bool FrameTransformGet_nwc_ros2::open(yarp::os::Searchable& config)
{
    if (!yarp::os::NetworkBase::checkNetwork()) {
        yCError(FRAMETRANSFORGETNWCROS2,"Error! YARP Network is not initialized");
        return false;
    }

    bool okGeneral = config.check("GENERAL");
    if(okGeneral)
    {
        yarp::os::Searchable& general_config = config.findGroup("GENERAL");
        if (general_config.check("refresh_interval"))  {m_refreshInterval = general_config.find("refresh_interval").asFloat64();}
    }

    m_ftContainer.m_timeout = m_refreshInterval;

    //ROS2 configuration
    if (config.check("ROS2"))
    {
        yCInfo(FRAMETRANSFORGETNWCROS2, "Configuring ROS2 params");
        Bottle ROS2_config = config.findGroup("ROS2");
        if(ROS2_config.check("ft_topic")) m_ftTopic = ROS2_config.find("ft_topic").asString();
        if(ROS2_config.check("ft_topic_static")) m_ftTopicStatic = ROS2_config.find("ft_topic_static").asString();
    }
    else
    {
        //no ROS2 options
        yCWarning(FRAMETRANSFORGETNWCROS2) << "ROS2 Group not configured";
    }

    m_subscriptionFtTimed = Ros2Init::get().node->create_subscription<tf2_msgs::msg::TFMessage>(m_ftTopic, 10,
                                                                                                std::bind(&FrameTransformGet_nwc_ros2::frameTransformTimedGet_callback,
                                                                                                this, _1));
    m_subscriptionFtStatic = Ros2Init::get().node->create_subscription<tf2_msgs::msg::TFMessage>(m_ftTopicStatic, 10,
                                                                                                 std::bind(&FrameTransformGet_nwc_ros2::frameTransformStaticGet_callback,
                                                                                                 this, _1));

    start();

    return true;
}

bool FrameTransformGet_nwc_ros2::close()
{
    rclcpp::shutdown();
    return true;
}

void FrameTransformGet_nwc_ros2::run()
{
    yCTrace(FRAMETRANSFORGETNWCROS2);

    rclcpp::spin(Ros2Init::get().node);

    return;
}

bool FrameTransformGet_nwc_ros2::getTransforms(std::vector<yarp::math::FrameTransform>& transforms) const
{
    std::lock_guard<std::mutex> lock(m_trf_mutex);
    if(!m_ftContainer.checkAndRemoveExpired())
    {
        yCError(FRAMETRANSFORGETNWCROS2,"Unable to remove expired transforms");
        return false;
    }
    m_ftContainer.getTransforms(transforms);
    return true;
}

void FrameTransformGet_nwc_ros2::frameTransformTimedGet_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    yCTrace(FRAMETRANSFORGETNWCROS2);
    updateBuffer(msg->transforms,false);
}

void FrameTransformGet_nwc_ros2::frameTransformStaticGet_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    yCTrace(FRAMETRANSFORGETNWCROS2);
    updateBuffer(msg->transforms,true);
}

double FrameTransformGet_nwc_ros2::yarpStampFromROS2(const builtin_interfaces::msg::Time& ros2Time)
{
    double yarpTime;
    double sec_part;
    double nsec_part;
    sec_part = (double)ros2Time.sec;
    nsec_part = ((double)ros2Time.nanosec)/1000000000.0;
    yarpTime = sec_part+nsec_part;

    return yarpTime;
}

void FrameTransformGet_nwc_ros2::ros2TransformToYARP(const geometry_msgs::msg::TransformStamped& input, yarp::math::FrameTransform& output, bool isStatic)
{
    output.translation.tX = input.transform.translation.x;
    output.translation.tY = input.transform.translation.y;
    output.translation.tZ = input.transform.translation.z;

    output.rotation.w() = input.transform.rotation.w;
    output.rotation.x() = input.transform.rotation.x;
    output.rotation.y() = input.transform.rotation.y;
    output.rotation.z() = input.transform.rotation.z;

    output.timestamp = yarpStampFromROS2(input.header.stamp);
    output.src_frame_id = input.header.frame_id;
    output.dst_frame_id = input.child_frame_id;
    output.isStatic = isStatic;
}

bool FrameTransformGet_nwc_ros2::updateBuffer(const std::vector<geometry_msgs::msg::TransformStamped>& transforms, bool areStatic)
{
    for (auto& it : transforms)
    {
        yarp::math::FrameTransform tempFT;
        ros2TransformToYARP(it,tempFT,areStatic);
        m_ftContainer.setTransform(tempFT);
    }
    return true;
}
