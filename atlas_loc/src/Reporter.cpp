//
//  Reporter.cpp
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <string>

#include <geometry_msgs/PointStamped.h>

#include "atlas_loc/atlas_types.h"
#include "atlas_loc/Reporter.h"

#include <atlas_msgs/Sample.h>
#include <atlas_msgs/Measurement.h>


Reporter::Reporter ()
{

}

Reporter::~Reporter()
{

}


void Reporter::initialize(ros::NodeHandle n)
{
    m_n=n;
    ROS_INFO("Initializing Reporter");
}

void Reporter::reportSample(const sample_t &s)
{
    //ROS_INFO(" %#lx: %lu", s.txeui, s.meas.size());

    if(m_samplePubs.find(s.txeui) == m_samplePubs.end())
    {
        m_seq.insert(std::make_pair(s.txeui, 0));

        std::stringstream stream;
        stream << std::hex << s.txeui;

        ros::Publisher pubSample = m_n.advertise<atlas_msgs::Sample>("/atlas/" + stream.str() + "/sample", 100);
        m_samplePubs.insert(std::make_pair(s.txeui, pubSample));
    }
    else
    {
        atlas_msgs::Sample sample_msg;
        sample_msg.hts = ros::Time::now();
        sample_msg.txId = s.txeui;
        sample_msg.seq = m_seq[s.txeui];

        for (auto it = s.meas.begin(); it != s.meas.end(); ++it)
        {
            atlas_msgs::Measurement meas_msg;
            meas_msg.rxId = it->first;
            meas_msg.ts = it->second.ts;
            meas_msg.toa = it->second.toa;
            meas_msg.lde.maxNoise = it->second.lde.maxNoise;
            meas_msg.lde.firstPathAmp1 = it->second.lde.firstPathAmp1;
            meas_msg.lde.stdNoise = it->second.lde.stdNoise;
            meas_msg.lde.firstPathAmp2 = it->second.lde.firstPathAmp2;
            meas_msg.lde.firstPathAmp3 = it->second.lde.firstPathAmp3;
            meas_msg.lde.maxGrowthCIR = it->second.lde.maxGrowthCIR;
            meas_msg.lde.rxPreamCount = it->second.lde.rxPreamCount;
            meas_msg.lde.firstPath = it->second.lde.firstPath;

            if(it->second.imu.magn[0] != 0)
            {
                sample_msg.imu.accel[0] = it->second.imu.accel[0];
                sample_msg.imu.accel[1] = it->second.imu.accel[1];
                sample_msg.imu.accel[2] = it->second.imu.accel[2];
                sample_msg.imu.gyro[0] = it->second.imu.gyro[0];
                sample_msg.imu.gyro[1] = it->second.imu.gyro[1];
                sample_msg.imu.gyro[2] = it->second.imu.gyro[2];
                sample_msg.imu.magn[0] = it->second.imu.magn[0];
                sample_msg.imu.magn[1] = it->second.imu.magn[1];
                sample_msg.imu.magn[2] = it->second.imu.magn[2];
            }

            sample_msg.meas.push_back(meas_msg);
        }

        m_samplePubs[s.txeui].publish(sample_msg);

        m_seq[s.txeui]++;
    }
}

void Reporter::reportPosition(const position_t &p)
{
    ROS_INFO(" %#lx: %.3f, %.3f, %.3f", p.eui, p.pos(0), p.pos(1), p.pos(2));

    if(m_pointPubs.find(p.eui) == m_pointPubs.end())
    {
        m_seq.insert(std::make_pair(p.eui, 0));

        std::stringstream stream;
        stream << std::hex << p.eui;

        ros::Publisher pubPoint = m_n.advertise<geometry_msgs::PointStamped>(stream.str() + "/point", 100);
        m_pointPubs.insert(std::make_pair(p.eui, pubPoint));
    }
    else
    {
        geometry_msgs::PointStamped point_msg;
        point_msg.header.frame_id = "map";
        point_msg.header.stamp = ros::Time::now();
        point_msg.header.seq = m_seq[p.eui];
        point_msg.point.x = p.pos(0);
        point_msg.point.y = p.pos(1);
        point_msg.point.z = p.pos(2);

        m_pointPubs[p.eui].publish(point_msg);

        m_seq[p.eui]++;
    }
}