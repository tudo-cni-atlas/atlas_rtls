//
//  Visualizer.cpp
//  atlas_viz
//
//  Created by Janis on 18.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#include "atlas/Visualizer.h"

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <iomanip>

#include "ros/ros.h"

#include <atlas_msgs/Sample.h>
#include <atlas_msgs/Measurement.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PRETTY MATRIX PRINT
std::string sep = "\n----------------------------------------\n";
Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
//std::cout << sep << "ta1:" << sep << ta1.format(HeavyFmt) << sep;


Visualizer::Visualizer ()
{

}

void Visualizer::initialize(ros::NodeHandle n)
{
    ROS_INFO("Initializing Visualizer");

    // Anchor node Configuration
    XmlRpc::XmlRpcValue anchors;
    n.getParam("/atlas/anchor", anchors);
    ROS_ASSERT(anchors.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = anchors.begin(); it != anchors.end(); ++it)
    {
        ROS_INFO("Found anchor %s", (it->first).c_str());
        uint64_t eui = std::stoul(it->first, nullptr, 16);

        std::vector<double> p;
        n.getParam("/atlas/anchor/" + it->first + "/pos", p);
        Eigen::Vector3d pos(p.at(0), p.at(1), p.at(2));
        m_anchorPositions.insert(std::pair<uint64_t, Eigen::Vector3d>(eui, pos));

        std::string s;
        n.getParam("/atlas/anchor/" + it->first + "/tdoaSyncEui", s);
        uint64_t seui = std::stoul(s, nullptr, 16);

        // ToDo: Visualize Sync Anchors
        //m_anchors.seuis.insert(std::pair<uint64_t, uint64_t>(eui, seui));

        ROS_INFO(" pos %f, %f, %f", p.at(0), p.at(1), p.at(2));
        ROS_INFO(" seui %ld", seui);
    }

    m_pubAnchors = n.advertise<visualization_msgs::Marker>("anchors", 1);
}

void Visualizer::visualizeAnchors(void)
{
    ROS_INFO(" Visualizing Anchors");

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "anchors";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.2;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.7;

    marker.lifetime = ros::Duration();

    for (auto it = m_anchorPositions.begin(); it != m_anchorPositions.end(); ++it)
    {
        geometry_msgs::Point p;
        p.x = it->second[0];
        p.y = it->second[1];
        p.z = it->second[2];
        marker.points.push_back(p);
    }

    m_pubAnchors.publish(marker);
}
