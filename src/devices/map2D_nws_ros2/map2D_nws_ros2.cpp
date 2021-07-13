/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <chrono>
#include <vector>
#include "Map2D_nws_ros2.h"
#include <yarp/dev/IMap2D.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/GenericVocabs.h>
#include <yarp/math/Math.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <mutex>
#include <cstdlib>
#include <fstream>
#include <yarp/rosmsg/impl/yarpRosHelper.h>

using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;
using namespace yarp::os;
using namespace std;
using namespace std::placeholders;

namespace {
YARP_LOG_COMPONENT(MAP2D_NWS_ROS2, "yarp.device.map2D_nws_ros2")
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

Map2D_nws_ros2::Map2D_nws_ros2()
{
    m_getMapName = "getMap";
    m_getMapByNameName = "getMapByName";
    m_markersName = "locationServerMarkers";
    m_rosCmdParserName = "rosCmdParser";
    m_spinned = false;
    m_currentMapName = "none";
}

bool Map2D_nws_ros2::attach(yarp::dev::PolyDriver* driver)
{
    if (driver->isValid())
    {
        driver->view(m_iMap2D);
        vector<string> maps;
        m_iMap2D->get_map_names(maps);
        m_currentMapName = maps[0];
    }

    if (nullptr == m_iMap2D)
    {
        yCError(MAP2D_NWS_ROS2, "Subdevice passed to attach method is invalid");
        return false;
    }

    return true;
}

bool Map2D_nws_ros2::detach()
{
    m_iMap2D = nullptr;
    return true;
}

bool Map2D_nws_ros2::open(yarp::os::Searchable &config)
{
    Property params;
    params.fromString(config.toString());

    if (!config.check("nws_name"))
    {
        m_rpcPortName = "/map2D_nws_ros/rpc";
    }
    else
    {
        m_rpcPortName = config.find("nws_name").asString()+"/rpc";
    }

    //subdevice handling
    if (config.check("subdevice"))
    {
        Property       p;
        p.fromString(config.toString(), false);
        p.put("device", config.find("subdevice").asString());

        if (!m_drv.open(p) || !m_drv.isValid())
        {
            yCError(MAP2D_NWS_ROS2) << "Failed to open subdevice.. check params";
            return false;
        }

        if (!attach(&m_drv))
        {
            yCError(MAP2D_NWS_ROS2) << "Failed to open subdevice.. check params";
            return false;
        }
    }
    else
    {
        yCInfo(MAP2D_NWS_ROS2) << "Waiting for device to attach";
    }

    //open rpc port
    if (!m_rpcPort.open(m_rpcPortName))
    {
        yCError(MAP2D_NWS_ROS2, "Failed to open port %s", m_rpcPortName.c_str());
        return false;
    }
    m_rpcPort.setReader(*this);

    //ROS configuration
    if (config.check("ROS"))
    {
        yCInfo(MAP2D_NWS_ROS2, "Configuring ROS params");
        Bottle ROS_config = config.findGroup("ROS");
        if(ROS_config.check("getmap")) m_getMapName = ROS_config.find("getmap").asString();
        if(ROS_config.check("getmapbyname")) m_getMapByNameName = ROS_config.find("getmapbyname").asString();
        if(ROS_config.check("roscmdparser")) m_rosCmdParserName = ROS_config.find("roscmdparser").asString();
        if(ROS_config.check("markers_pub")) m_markersName = ROS_config.find("markers_pub").asString();
        m_ros2Service_getMap = Ros2Init::get().node->create_service<nav_msgs::srv::GetMap>(m_getMapName,
                                                                                           std::bind(&Map2D_nws_ros2::getMapCallback,this,_1,_2,_3));
        m_ros2Service_getMapByName = Ros2Init::get().node->create_service<map2d_nws_ros2_msgs::srv::GetMapByName>(m_getMapByNameName,
                                                                                                                  std::bind(&Map2D_nws_ros2::getMapByNameCallback,this,_1,_2,_3));
        m_ros2Service_rosCmdParser = Ros2Init::get().node->create_service<test_msgs::srv::BasicTypes>(m_rosCmdParserName,
                                                                                                      std::bind(&Map2D_nws_ros2::rosCmdParserCallback,this,_1,_2,_3));
    }
    else
    {
        //no ROS options
        yCWarning(MAP2D_NWS_ROS2) << "ROS Group not configured";
    }

    start();

    return true;
}


void Map2D_nws_ros2::run()
{
    if(!m_spinned)  //This is just a temporary solution.
    {
        rclcpp::spin(Ros2Init::get().node);
        m_spinned = true;
    }
}

bool Map2D_nws_ros2::close()
{
    yCTrace(MAP2D_NWS_ROS2, "Close");
    m_rpcPort.close();
    rclcpp::shutdown();
    return true;
}

bool Map2D_nws_ros2::read(yarp::os::ConnectionReader& connection)
{
    yCWarning(MAP2D_NWS_ROS2) << "not yet implemented";

    std::lock_guard<std::mutex> lock(m_mutex);
    yarp::os::Bottle in;
    yarp::os::Bottle out;
    bool ok = in.read(connection);
    if (!ok) return false;

    //parse string command
    if(in.get(0).isString())
    {
      //  parse_string_command(in, out);
    }
    // parse vocab command
    else if(in.get(0).isVocab())
    {
   //     parse_vocab_command(in, out);
    }

    yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
    if (returnToSender != nullptr)
    {
        out.write(*returnToSender);
    }
    else
    {
        yCError(MAP2D_NWS_ROS2) << "Invalid return to sender";
    }
    return true;
}

bool Map2D_nws_ros2::updateVizMarkers()
{
    if (!m_ros2Publisher_markers)
    {
        m_ros2Publisher_markers = Ros2Init::get().node->create_publisher<visualization_msgs::msg::MarkerArray>(m_markersName, 10);
    }
    builtin_interfaces::msg::Duration dur;
    dur.sec = 0xFFFFFFFF;
    double yarpTimeStamp = yarp::os::Time::now();
    uint64_t time;
    uint64_t sec_part;
    rclcpp::Time ret;
    time = (uint64_t)(yarpTimeStamp * 1000000000UL);
    sec_part = (time / 1000000000UL);
    if (sec_part > std::numeric_limits<unsigned int>::max())
    {
        yCWarning(MAP2D_NWS_ROS2) << "Timestamp exceeded the 64 bit representation, resetting it to 0";
        sec_part = 0;
    }
    visualization_msgs::msg::Marker marker;
    builtin_interfaces::msg::Time    tt;
    yarp::sig::Vector         rpy(3);
    yarp::math::Quaternion    q;
    visualization_msgs::msg::MarkerArray markers;

    std::vector<std::string> locations;
    int count = 1;
    m_iMap2D->getLocationsList(locations);
    for (auto it : locations)
    {
        yarp::dev::Nav2D::Map2DLocation loc;
        m_iMap2D->getLocation(it, loc);

        if(loc.map_id != m_currentMapName && m_currentMapName != "none")
        {
            continue;
        }

        rpy[0] = 0; //x
        rpy[1] = 0; //y
        rpy[2] = loc.theta / 180 * PI; //z
        yarp::sig::Matrix m = yarp::math::rpy2dcm(rpy);
        q.fromRotationMatrix(m);

        marker.header.frame_id    = "map";
        tt.sec                    = (yarp::os::NetUint32) sec_part;;
        marker.header.stamp       = tt;
        marker.ns                 = m_markersName+"_ns";
        marker.id                 = count;
        marker.type               = visualization_msgs::msg::Marker::ARROW;//yarp::rosmsg::visualization_msgs::Marker::ARROW;
        marker.action             = visualization_msgs::msg::Marker::ADD;//yarp::rosmsg::visualization_msgs::Marker::ADD;
        marker.pose.position.x    = loc.x;
        marker.pose.position.y    = loc.y;
        marker.pose.position.z    = 0;
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        marker.scale.x            = 1;
        marker.scale.y            = 0.1;
        marker.scale.z            = 0.1;
        marker.color.a            = 1.0;
        marker.color.r            = 0.0;
        marker.color.g            = 1.0;
        marker.color.b            = 0.0;
        marker.lifetime           = dur;
        marker.text               = it;
        markers.markers.push_back(marker);
        count++;
    }

    m_ros2Publisher_markers->publish(markers);
    return true;
}


void Map2D_nws_ros2::getMapCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                    const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
                                    std::shared_ptr<nav_msgs::srv::GetMap::Response> response)
{
    yCWarning(MAP2D_NWS_ROS2) << "Not yet implemented";
    nav_msgs::msg::OccupancyGrid mapToGo;
    mapToGo.header.frame_id = "map";

    response->map = mapToGo;
}

void Map2D_nws_ros2::rosCmdParserCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                     const std::shared_ptr<test_msgs::srv::BasicTypes::Request> request,
                                     std::shared_ptr<test_msgs::srv::BasicTypes::Response> response)
{
    if(request->string_value == "updatemarkerviz")
    {
        if(updateVizMarkers()) response->set__string_value("done");
        else response->set__string_value("error");
    }
    else
    {
        response->set__string_value("unknown_command");
    }
}

//void Map2D_nws_ros2::prepareMapMsg(MapGrid2D inputMap, nav_msgs::msg::OccupancyGrid &outputMsg)
void Map2D_nws_ros2::getMapByNameCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                          const std::shared_ptr<map2d_nws_ros2_msgs::srv::GetMapByName::Request> request,
                                          std::shared_ptr<map2d_nws_ros2_msgs::srv::GetMapByName::Response> response)
{
    if (!m_ros2Publisher_map)
    {
        m_ros2Publisher_map = Ros2Init::get().node->create_publisher<nav_msgs::msg::OccupancyGrid>(m_getMapByNameName+"/pub", 10);
    }
    nav_msgs::msg::OccupancyGrid mapToGo;
    nav_msgs::msg::MapMetaData metaToGo;
    mapToGo.header.frame_id = "map";
    MapGrid2D theMap;
    if(!m_iMap2D->get_map(request->name,theMap))
    {
        mapToGo.header.frame_id = "invalid_frame";
        response->map = mapToGo;
        return;
    }
    mapToGo.info.map_load_time = Ros2Init::get().node->get_clock()->now();
    mapToGo.header.stamp = Ros2Init::get().node->get_clock()->now();
    mapToGo.info.height = theMap.height();
    mapToGo.info.width = theMap.width();

    double DEG2RAD = PI/180.0;
    double tmp=0;
    theMap.getResolution(tmp);
    mapToGo.info.resolution=tmp;
    double x, y, t;
    theMap.getOrigin(x,y,t);
    mapToGo.info.origin.position.x=x;
    mapToGo.info.origin.position.y=y;
    yarp::math::Quaternion q;
    yarp::sig::Vector v(4);
    v[0]=0; v[1]=0; v[2]=1; v[3]=t*DEG2RAD;
    q.fromAxisAngle(v);
    mapToGo.info.origin.orientation.x = q.x();
    mapToGo.info.origin.orientation.y = q.y();
    mapToGo.info.origin.orientation.z = q.z();
    mapToGo.info.origin.orientation.w = q.w();
    mapToGo.data.resize(theMap.width()*theMap.height());
    int index=0;
    yarp::dev::Nav2D::XYCell cell;
    for (cell.y=theMap.height(); cell.y-- > 0;)
    {
      for (cell.x=0; cell.x<theMap.width(); cell.x++)
      {
        theMap.getOccupancyData(cell,tmp);
        mapToGo.data[index++]=(int)tmp;
      }
    }

    response->map = mapToGo;
    m_currentMapName = request->name;

    metaToGo.map_load_time = mapToGo.info.map_load_time;
    metaToGo.height = theMap.height();
    metaToGo.width = theMap.width();
    metaToGo.origin = mapToGo.info.origin;
    metaToGo.resolution = mapToGo.info.resolution;

    if (m_ros2Publisher_map)
    {
        m_ros2Publisher_map->publish(mapToGo);
    }
}
