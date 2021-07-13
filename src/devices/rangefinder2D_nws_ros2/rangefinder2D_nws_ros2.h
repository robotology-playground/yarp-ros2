/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_ROS2_RANGEFINDER2D_NWS_ROS2_H
#define YARP_ROS2_RANGEFINDER2D_NWS_ROS2_H

#include <yarp/dev/WrapperSingle.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/PeriodicThread.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <mutex>


class Ros2Init
{
public:
    Ros2Init();

    std::shared_ptr<rclcpp::Node> node;

    static Ros2Init& get();
};


class Rangefinder2D_nws_ros2 :
        public yarp::dev::DeviceDriver,
        public yarp::os::PeriodicThread,
        public yarp::dev::WrapperSingle
{
public:
    Rangefinder2D_nws_ros2();
    Rangefinder2D_nws_ros2(const Rangefinder2D_nws_ros2&) = delete;
    Rangefinder2D_nws_ros2(Rangefinder2D_nws_ros2&&) noexcept = delete;
    Rangefinder2D_nws_ros2& operator=(const Rangefinder2D_nws_ros2&) = delete;
    Rangefinder2D_nws_ros2& operator=(Rangefinder2D_nws_ros2&&) noexcept = delete;
    ~Rangefinder2D_nws_ros2() override = default;

    //WrapperSingle
    bool attach(yarp::dev::PolyDriver* driver) override;
    bool detach() override;
    
    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;

private:
    yarp::dev::PolyDriver m_driver;
    yarp::dev::IRangefinder2D *m_iDevice =nullptr;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_publisher;
    std::string m_topic;
    bool m_isDeviceOwned = false;
    
    double m_minAngle, m_maxAngle;
    double m_minDistance, m_maxDistance;
    double m_resolution;
    double m_period;
    std::string m_frame_id;
};

#endif // YARP_ROS2_ROS2TEST_H
