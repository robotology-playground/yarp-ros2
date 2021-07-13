/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_CONTROLBOARD_NWS_ROS2_H
#define YARP_DEV_CONTROLBOARD_NWS_ROS2_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/WrapperSingle.h>
#include <yarp/os/PeriodicThread.h>

#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IAxisInfo.h>

#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <mutex>

class Ros2Init
{
public:
    Ros2Init();

    std::shared_ptr<rclcpp::Node> node;

    static Ros2Init& get();
};



/**
 *  @ingroup dev_impl_wrapper
 *
 * \brief `controlBoard_nws_ros`: A controlBoard network wrapper server for ROS2.
 *
 * \section controlBoard_nws_ros_device_parameters Description of input parameters
 *
 *  Parameters required by this device are:
 * | Parameter name | SubParameter   | Type    | Units          | Default Value | Required                    | Description                                                       | Notes |
 * |:--------------:|:--------------:|:-------:|:--------------:|:-------------:|:--------------------------: |:-----------------------------------------------------------------:|:-----:|
 * | nodeName       |      -         | string  | -              |   -           | Yes                         | set the name for ROS node                                         | must start with a leading '/' |
 * | topicName      |      -         | string  | -              |   -           | Yes                         | set the name for ROS topic                                        | must start with a leading '/' |
 * | period         |      -         | double  | s              |   0.02        | No                          | refresh period of the broadcasted values in s                     | optional, default 20ms |
 * | subdevice      |      -         | string  | -              |   -           | No                          | name of the subdevice to instantiate                              | when used, parameters for the subdevice must be provided as well |
 *
 * ROS message type used is sensor_msgs/JointState.msg (http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html)
 */

class ControlBoard_nws_ros2 :
        public yarp::dev::DeviceDriver,
        public yarp::os::PeriodicThread,
        public yarp::dev::WrapperSingle
{
private:
    sensor_msgs::msg::JointState ros_struct;

    yarp::sig::Vector times; // time for each joint

    std::vector<std::string> jointNames; // name of the joints
    std::string nodeName;                // name of the rosNode
    std::string topicName;               // name of the rosTopic

//     yarp::os::Node* node; // ROS node
    std::uint32_t counter {0}; // incremental counter in the ROS message

//     yarp::os::PortWriterBuffer<yarp::rosmsg::sensor_msgs::JointState> rosOutputState_buffer; // Buffer associated to the ROS topic
//     yarp::os::Publisher<yarp::rosmsg::sensor_msgs::JointState> rosPublisherPort;             // Dedicated ROS topic publisher

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_publisher;

    static constexpr double default_period = 0.02; // s
    double period {default_period};

    std::string partName; // to open ports and print more detailed debug messages
    yarp::os::Stamp time; // envelope to attach to the state port

    yarp::dev::DeviceDriver* subdevice_ptr{nullptr};
    bool subdevice_owned {true};
    size_t subdevice_joints {0};
    bool subdevice_ready = false;

    yarp::dev::IPositionControl* iPositionControl{nullptr};
    yarp::dev::IEncodersTimed* iEncodersTimed{nullptr};
    yarp::dev::ITorqueControl* iTorqueControl{nullptr};
    yarp::dev::IAxisInfo* iAxisInfo{nullptr};

    bool setDevice(yarp::dev::DeviceDriver* device, bool owned);
    bool openAndAttachSubDevice(yarp::os::Property& prop);

    void closeDevice();
    void closePorts();
    bool updateAxisName();

public:
    ControlBoard_nws_ros2();
    ControlBoard_nws_ros2(const ControlBoard_nws_ros2&) = delete;
    ControlBoard_nws_ros2(ControlBoard_nws_ros2&&) = delete;
    ControlBoard_nws_ros2& operator=(const ControlBoard_nws_ros2&) = delete;
    ControlBoard_nws_ros2& operator=(ControlBoard_nws_ros2&&) = delete;
    ~ControlBoard_nws_ros2() override = default;

    // yarp::dev::DeviceDriver
    bool close() override;
    bool open(yarp::os::Searchable& prop) override;

    // yarp::dev::IWrapper
    bool attach(yarp::dev::PolyDriver* poly) override;
    bool detach() override;

    // yarp::os::PeriodicThread
    void run() override;
};


/*

class MinimalPublisher
{
public:
    MinimalPublisher(const std::string& topicname);
};


class Ros2Test :
        public yarp::dev::DeviceDriver,
        public yarp::os::PeriodicThread
{
public:
    Ros2Test();
    Ros2Test(const Ros2Test&) = delete;
    Ros2Test(Ros2Test&&) noexcept = delete;
    Ros2Test& operator=(const Ros2Test&) = delete;
    Ros2Test& operator=(Ros2Test&&) noexcept = delete;
    ~Ros2Test() override = default;

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;

private:
    std::string m_topic;
    size_t m_count {0};
};*/

#endif // YARP_DEV_CONTROLBOARD_NWS_ROS2_H
