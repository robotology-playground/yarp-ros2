/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "subscriber_example.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

using namespace std::chrono_literals;
using namespace std::placeholders;

YARP_LOG_COMPONENT(SUBEX, "yarp.ros2.subEx", yarp::os::Log::TraceType);


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


SubEx::SubEx() :
    m_topic("ros2test_topic")
{
}

bool SubEx::open(yarp::os::Searchable& config)
{
    yCTrace(SUBEX);
    if(config.check("ROS2"))
    {
        yarp::os::Bottle generalProp = config.findGroup("ROS2");
        if(generalProp.check("topic")) {m_topic = generalProp.find("topic").asString();}
    }

    m_subscription = Ros2Init::get().node->create_subscription<std_msgs::msg::String>(m_topic, 10, std::bind(&SubEx::simple_callback, this, _1));

    start();
    return true;
}

bool SubEx::close()
{
    yCTrace(SUBEX);
    yCInfo(SUBEX, "SubEx::close");
    rclcpp::shutdown();
    return true;
}

void SubEx::run()
{
    yCTrace(SUBEX);
    rclcpp::spin(Ros2Init::get().node);
}

void SubEx::simple_callback(const std_msgs::msg::String::SharedPtr msg) const
{
    yCInfo(SUBEX) << "Message received " << msg->data;
}
