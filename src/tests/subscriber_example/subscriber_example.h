/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_ROS2_SUBEX_H
#define YARP_ROS2_SUBEX_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/Thread.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <mutex>

class Ros2Init
{
public:
    Ros2Init();

    std::shared_ptr<rclcpp::Node> node;

    static Ros2Init& get();
};


class SubEx :
        public yarp::dev::DeviceDriver,
        public yarp::os::Thread
{
public:
    SubEx();
    SubEx(const SubEx&) = delete;
    SubEx(SubEx&&) noexcept = delete;
    SubEx& operator=(const SubEx&) = delete;
    SubEx& operator=(SubEx&&) noexcept = delete;
    ~SubEx() override = default;

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // Thread
    void run() override;

    // Inner functions
    void simple_callback(const std_msgs::msg::String::SharedPtr msg) const;

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subscription;
    std::string                                            m_topic;
};

#endif // YARP_ROS2_SUBEX_H