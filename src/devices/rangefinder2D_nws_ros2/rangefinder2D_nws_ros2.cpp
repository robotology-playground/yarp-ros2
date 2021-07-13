/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "Rangefinder2D_nws_ros2.h"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

using namespace std::chrono_literals;
using namespace yarp::os;
using namespace yarp::dev;


YARP_LOG_COMPONENT(RANGEFINDER2D_NWS_ROS2, "yarp.ros2.rangefinder2D_nws_ros2", yarp::os::Log::TraceType);


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


Rangefinder2D_nws_ros2::Rangefinder2D_nws_ros2() :
        yarp::os::PeriodicThread(0.01)
{
}

bool Rangefinder2D_nws_ros2::attach(yarp::dev::PolyDriver* driver)
{
    if (driver->isValid())
    {
        driver->view(m_iDevice);
    }

    //attach the hardware device
    if (nullptr == m_iDevice)
    {
        yCError(RANGEFINDER2D_NWS_ROS2, "Subdevice passed to attach method is invalid");
        return false;
    }

    //get information/parameters from the hardware device etc
    if(!m_iDevice->getDistanceRange(m_minDistance, m_maxDistance))
    {
        yCError(RANGEFINDER2D_NWS_ROS2) << "Laser device does not provide min & max distance range.";
        return false;
    }

    if(!m_iDevice->getScanLimits(m_minAngle, m_maxAngle))
    {
        yCError(RANGEFINDER2D_NWS_ROS2) << "Laser device does not provide min & max angle scan range.";
        return false;
    }

    if (!m_iDevice->getHorizontalResolution(m_resolution))
    {
        yCError(RANGEFINDER2D_NWS_ROS2) << "Laser device does not provide horizontal resolution ";
        return false;
    }
    
   return true;
}

bool Rangefinder2D_nws_ros2::detach()
{
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    m_iDevice = nullptr;
    return true;
}

void Rangefinder2D_nws_ros2::run()
{
    yCTrace(RANGEFINDER2D_NWS_ROS2);
    auto message = std_msgs::msg::String();
    
    if (m_iDevice!=nullptr)
    {
        bool ret = true;
        IRangefinder2D::Device_status status;
        yarp::sig::Vector ranges;
        ret &= m_iDevice->getRawData(ranges);
        ret &= m_iDevice->getDeviceStatus(status);
        
        if (ret)
        {
            int ranges_size = ranges.size();

            sensor_msgs::msg::LaserScan rosData;

            rosData.header.stamp = Ros2Init::get().node->get_clock()->now();    //@@@@@@@@@@@ CHECK HERE: simulation time?
            rosData.header.frame_id = m_frame_id;
            rosData.angle_min = m_minAngle * M_PI / 180.0;
            rosData.angle_max = m_maxAngle * M_PI / 180.0;
            rosData.angle_increment = m_resolution * M_PI / 180.0;
            rosData.time_increment = 0;             // all points in a single scan are considered took at the very same time
            rosData.scan_time = getPeriod();        // time elapsed between two successive readings
            rosData.range_min = m_minDistance;
            rosData.range_max = m_maxDistance;
            rosData.ranges.resize(ranges_size);
            rosData.intensities.resize(ranges_size);

            for (int i = 0; i < ranges_size; i++)
            {
                // in yarp, NaN is used when a scan value is missing. For example when the angular range of the rangefinder is smaller than 360.
                // is ros, NaN is not used. Hence this check replaces NaN with inf.
                if (std::isnan(ranges[i]))
                {
                   rosData.ranges[i] = std::numeric_limits<double>::infinity();
                   rosData.intensities[i] = 0.0;
                }
                else
                {
                   rosData.ranges[i] = ranges[i];
                   rosData.intensities[i] = 0.0;
                }
            }
            m_publisher->publish(rosData);
        }
        else
        {
            yCError(RANGEFINDER2D_NWS_ROS2, "Sensor returned error");
        }
    }
}

bool Rangefinder2D_nws_ros2::open(yarp::os::Searchable &config)
{
    if(config.check("subdevice"))
    {
        Property       p;
        p.fromString(config.toString(), false);
        p.put("device", config.find("subdevice").asString());

        if(!m_driver.open(p) || !m_driver.isValid())
        {
            yCError(RANGEFINDER2D_NWS_ROS2) << "Failed to open subdevice.. check params";
            return false;
        }

        if(!attach(&m_driver))
        {
            yCError(RANGEFINDER2D_NWS_ROS2) << "Failed to open subdevice.. check params";
            return false;
        }
        m_isDeviceOwned = true;
    }
 
    //wrapper params
    m_topic    = config.check("topic",  yarp::os::Value("laser_topic"), "Name of the ROS2 topic").asString();
    m_frame_id = config.check("frame",  yarp::os::Value("laser_frame"), "Name of the frameId").asString();
    m_period   = config.check("period", yarp::os::Value(0.010), "Period of the thread").asFloat64();
       
    //create the topic
    yCTrace(RANGEFINDER2D_NWS_ROS2);

    m_publisher = Ros2Init::get().node->create_publisher<sensor_msgs::msg::LaserScan>(m_topic, 10);
    yCInfo(RANGEFINDER2D_NWS_ROS2, "Opened topic: %s", m_topic.c_str());
        
    //start the publishig thread
    setPeriod(m_period);
    start();
    return true;
}

bool Rangefinder2D_nws_ros2::close()
{
    return true;
}
