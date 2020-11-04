//
//  Assembly.cpp
//  atlas core
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <map>
#include <string>

#include "atlas_core/atlas_types.h"
#include "atlas_core/Assembly.h"


typedef enum
{
    CFG_ROLE_LISTENER = 0,
    CFG_ROLE_TAG,
    CFG_ROLE_ANCHOR,
    CFG_ROLE_TAG_TDOA,
    CFG_ROLE_NUM_MODES,
    CFG_ROLE_SYNC_ANCHOR
} cfgRole_t;

Assembly::Assembly()
{

}

Assembly::~Assembly()
{

}

void Assembly::initialize(ros::NodeHandle n)
{
    ROS_INFO("Initializing Assembly...");

    XmlRpc::XmlRpcValue anchors;
    n.getParam("/atlas/anchor", anchors);
    ROS_ASSERT(anchors.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = anchors.begin(); it != anchors.end(); ++it)
    {
        ROS_INFO("Found anchor %s", (it->first).c_str());
        uint64_t eui = std::stoul(it->first, nullptr, 16);
        std::stringstream euis;
        euis << std::hex << eui;
        m_subToa.insert(std::make_pair(eui, n.subscribe("/atlas/anchor/" + euis.str() + "/toa", 100, &Assembly::toaCallback, this)));
        m_subToaLde.insert(std::make_pair(eui, n.subscribe("/atlas/anchor/" + euis.str() + "/toaLde", 100, &Assembly::toaLdeCallback, this)));
        m_subToaImu.insert(std::make_pair(eui, n.subscribe("/atlas/anchor/" + euis.str() + "/toaImu", 100, &Assembly::toaImuCallback, this)));
        m_subToaImuLde.insert(std::make_pair(eui, n.subscribe("/atlas/anchor/" + euis.str() + "/toaImuLde", 100, &Assembly::toaImuLdeCallback, this)));
    }
}

void Assembly::toaCallback(const atlas_msgs::Toa& msg)
{
    measurement_t meas;
    meas.hts = ros::Time::now();
    meas.ts = msg.ts;
    meas.seq = msg.seq;

    m_measurements[msg.txId][meas.seq].insert(std::make_pair(msg.rxId, meas));
}

void Assembly::toaLdeCallback(const atlas_msgs::ToaLde& msg)
{
    measurement_t meas;
    meas.hts = ros::Time::now();
    meas.ts = msg.ts;
    meas.seq = msg.seq;
    meas.lde.maxNoise = msg.lde.maxNoise;
    meas.lde.firstPathAmp1 = msg.lde.firstPathAmp1;
    meas.lde.stdNoise = msg.lde.stdNoise;
    meas.lde.firstPathAmp2 = msg.lde.firstPathAmp2;
    meas.lde.firstPathAmp3 = msg.lde.firstPathAmp3;
    meas.lde.maxGrowthCIR = msg.lde.maxGrowthCIR;
    meas.lde.rxPreamCount = msg.lde.rxPreamCount;
    meas.lde.firstPath = msg.lde.firstPath;

    m_measurements[msg.txId][meas.seq].insert(std::make_pair(msg.rxId, meas));
}

void Assembly::toaImuCallback(const atlas_msgs::ToaImu& msg)
{
    measurement_t meas;
    meas.hts = ros::Time::now();
    meas.ts = msg.ts;
    meas.seq = msg.seq;
    meas.imu.accel(0) = msg.imu.accel[0];
    meas.imu.accel(1) = msg.imu.accel[1];
    meas.imu.accel(2) = msg.imu.accel[2];
    meas.imu.gyro(0) = msg.imu.gyro[0];
    meas.imu.gyro(1) = msg.imu.gyro[1];
    meas.imu.gyro(2) = msg.imu.gyro[2];
    meas.imu.magn(0) = msg.imu.magn[0];
    meas.imu.magn(1) = msg.imu.magn[1];
    meas.imu.magn(2) = msg.imu.magn[2];

    m_measurements[msg.txId][meas.seq].insert(std::make_pair(msg.rxId, meas));
}

void Assembly::toaImuLdeCallback(const atlas_msgs::ToaImuLde& msg)
{
    measurement_t meas;
    meas.hts = ros::Time::now();
    meas.ts = msg.ts;
    meas.seq = msg.seq;
    meas.imu.accel(0) = msg.imu.accel[0];
    meas.imu.accel(1) = msg.imu.accel[1];
    meas.imu.accel(2) = msg.imu.accel[2];
    meas.imu.gyro(0) = msg.imu.gyro[0];
    meas.imu.gyro(1) = msg.imu.gyro[1];
    meas.imu.gyro(2) = msg.imu.gyro[2];
    meas.imu.magn(0) = msg.imu.magn[0];
    meas.imu.magn(1) = msg.imu.magn[1];
    meas.imu.magn(2) = msg.imu.magn[2];

    meas.lde.maxNoise = msg.lde.maxNoise;
    meas.lde.firstPathAmp1 = msg.lde.firstPathAmp1;
    meas.lde.stdNoise = msg.lde.stdNoise;
    meas.lde.firstPathAmp2 = msg.lde.firstPathAmp2;
    meas.lde.firstPathAmp3 = msg.lde.firstPathAmp3;
    meas.lde.maxGrowthCIR = msg.lde.maxGrowthCIR;
    meas.lde.rxPreamCount = msg.lde.rxPreamCount;
    meas.lde.firstPath = msg.lde.firstPath;

    m_measurements[msg.txId][meas.seq].insert(std::make_pair(msg.rxId, meas));
}

void Assembly::extractSamples(std::vector<sample_t> *samples)
{
    // go through measurements and schedule latest
    ros::Time now = ros::Time::now();

    // go through txeuis
    for (auto it_txeui = m_measurements.begin(); it_txeui != m_measurements.end(); ++it_txeui)
    {
        // go through sequence numbers
        for (auto it_seq = it_txeui->second.begin(); it_seq != it_txeui->second.end();)
        {
            bool dispatch = false;

            // go through rxeuis
            for (auto it_rxeui = it_seq->second.begin(); it_rxeui != it_seq->second.end(); ++it_rxeui)
            {
                // go through
                ros::Duration d = now - it_rxeui->second.hts;
                if(d > ros::Duration(0.1))
                {
                    dispatch = true;
                }
            }

            if(dispatch)
            {
                // linearize sequence number
                int64_t seq = it_seq->first;
                uint64_t txeui = it_txeui->first;

                if(m_lastSequence.find(txeui) == m_lastSequence.end())
                {
                    m_lastSequence.insert(std::make_pair(txeui, seq));
                }
                else
                {
                    int64_t lastseq = m_lastSequence[txeui];
                    int64_t res = lastseq % 256;

                    if(seq - res > 0)
                    {
                        m_lastSequence[txeui] = lastseq + (seq - res);
                        //ROS_DEBUG("Linearizing + seq: %ld, lastseq: %ld, res: %ld", seq, lastseq, res);
                    }
                    else if(seq - res < 0)
                    {
                        m_lastSequence[txeui] = lastseq + (seq - res) + 256;
                        //ROS_DEBUG("Linearizing - seq: %ld, lastseq: %ld, res: %ld", seq, lastseq, res);
                    }

                    if(m_lastSequence[txeui] - lastseq > 1)
                    {
                        ROS_WARN("Missed previous tag packet(s) %lu, txeui: %zu", m_lastSequence[txeui], txeui);
                    }
                }

                ROS_DEBUG("Dispatching sample: %#lx, seq: %lu, size: %lu", txeui, m_lastSequence[txeui], it_seq->second.size());

                sample_t s;
                s.hts = ros::Time::now();
                s.txeui = txeui;
                s.seq = m_lastSequence[txeui];
                s.meas = it_seq->second;

                samples->push_back(s);
                it_txeui->second.erase(it_seq++);
            }
            else
            {
                ++it_seq;
            }
        }
    }
}
