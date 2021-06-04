/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_FRAMETRANSFORMSETNWCROS2_H
#define YARP_DEV_FRAMETRANSFORMSETNWCROS2_H


#include <yarp/os/Network.h>
#include <yarp/dev/IFrameTransformStorage.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/WrapperSingle.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <frameTransformContainer.h>
#include <mutex>
#include <map>


#define ROS2TOPICNAME_TF "/tf"
#define ROS2TOPICNAME_TF_STATIC "/tf_static"

/*
 * \section FrameTransformSet_nwc_ros2_device_parameters Description of input parameters
 *
 *   Parameters required by this device are:
 * | Parameter name | SubParameter         | Type    | Units          | Default Valu          | Required     | Description                                                                                             |
 * |:--------------:|:--------------------:|:-------:|:--------------:|:---------------------:|:-----------: |:-------------------------------------------------------------------------------------------------------:|
 * | GENERAL        |      -               | group   | -              | -                     | No           |                                                                                                         |
 * | -              | period               | double  | seconds        | 0.01                  | No           | The PeriodicThread period in seconds                                                                    |
 * | -              | refresh_interval     | double  | seconds        | 0.1                   | No           | The time interval outside which timed fts will be deleted                                               |
 * | -              | asynch_pub           | int     | -              | 1                     | No           | If 1, the fts will be published not only every "period" seconds but also when set functions are called  |
 * | ROS2           |      -               | group   | -              | -                     | No           |                                                                                                         |
 * | -              | ft_topic             | string  | -              | /tf                   | No           | The name of the ROS2 topic on which fts will be published                                               |
 * | -              | ft_topic_static      | string  | -              | /tf_static            | No           | The name of the ROS2 topic on which static fts will be published                                        |
 *
 * Some example of configuration files:
 *
 * Example of configuration file using .ini format.
 *
 * \code{.unparsed}
 * device frameTransformSet_nwc_ros2
 * [GENERAL]
 * period 0.05
 * refresh_interval 0.2
 * asynch_pub 1
 * [ROS2]
 * ft_topic /tf
 * ft_topic_static /tf_static
 * ft_node /tfNode
 * \endcode
 */

class Ros2Init
{
public:
    Ros2Init();

    std::shared_ptr<rclcpp::Node> node;

    static Ros2Init& get();
};


class FrameTransformSet_nwc_ros2 :
    public yarp::dev::DeviceDriver,
    public yarp::os::PeriodicThread,
    public yarp::dev::IFrameTransformStorageSet
{
public:
    FrameTransformSet_nwc_ros2(double tperiod=0.010);
    ~FrameTransformSet_nwc_ros2()=default;

    //DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    //periodicThread
    void run() override;

    //IFrameTransformStorageSet interface
    bool setTransforms(const std::vector<yarp::math::FrameTransform>& transforms) override;
    bool setTransform(const yarp::math::FrameTransform& transform) override;

    //own
    bool publishFrameTransforms();
    void ros2TimeFromYarp(double yarpTime, builtin_interfaces::msg::Time& ros2Time);
    void yarpTransformToROS2Transform(const yarp::math::FrameTransform &input, geometry_msgs::msg::TransformStamped& output);

private:
    mutable std::mutex                                      m_trf_mutex;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr  m_publisherFtTimed;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr  m_publisherFtStatic;
    std::string                                             m_ftTopic{ROS2TOPICNAME_TF};
    std::string                                             m_ftTopicStatic{ROS2TOPICNAME_TF};
    FrameTransformContainer                                 m_ftContainer;
    double                                                  m_period;
    double                                                  m_refreshInterval{0.1};
    bool                                                    m_asynchPub{true};
};

#endif // YARP_DEV_FRAMETRANSFORMSETNWCROS2_H
