//
//  ClockCorrection.cpp
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

#include "atlas_core/atlas_types.h"
#include "atlas_core/ClockCorrection.h"

#include <atlas_msgs/Trim.h>


ClockModel::ClockModel ()
{
    m_lastSync = 0;
    m_lastOffset = 0;
    m_lastDrift = 0;
}

ClockModel::~ClockModel()
{

}

void ClockModel::processSynchronizationFrame(double toa, double ref)
{
    double offset = toa - ref;
    double drift = (offset - m_lastOffset) / (toa - m_lastSync);

    //std::cout << "Sync - reference: " << std::setprecision(2) << std::fixed << ref;
    //std::cout << std::setprecision(2) << std::fixed << "s, offset: " << offset*1000;
    //std::cout << std::setprecision(2) << std::fixed << "ms, drift: " << drift*1000000000 << "ns/s" << std::endl;

    m_lastSync = toa;
    m_lastOffset = offset;
    m_lastDrift = drift;
    m_lastUpdate = ros::Time::now();
}

double ClockModel::getCorrectedTOA(double toa) const
{
    double td = toa - m_lastSync;
    double correction = m_lastOffset + m_lastDrift * td;
    double corrected = toa - correction;

    //std::cout << "Loc - toa: " << std::setprecision(3) << std::fixed << toa;
    //std::cout << std::setprecision(2) << std::fixed << "s, td: " << td*1000;
    //std::cout << std::setprecision(2) << std::fixed << "ms, drift*td: " << m_lastDrift*td*1000000000;
    //std::cout << std::setprecision(9) << std::fixed << "ns, corrected: " << corrected << std::endl;

    return corrected;
}

double ClockModel::getLastOffset() const
{
    return m_lastOffset;
}

double ClockModel::getLastDrift() const
{
    return m_lastDrift;
}

ros::Time ClockModel::getLastUpdate() const
{
    return m_lastUpdate;
}


ClockCorrection::ClockCorrection ()
{

}

ClockCorrection::~ClockCorrection()
{

}

void ClockCorrection::initialize(ros::NodeHandle n)
{
    ROS_INFO("Initializing Clock Correction...");

    std::map<uint64_t, uint64_t> seuis;
    std::map<uint64_t, uint8_t> xtalts;

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
        seuis.insert(std::pair<uint64_t, uint64_t>(eui, seui));

        int xtalt;
        n.getParam("/atlas/anchor/" + it->first + "/xtalt", xtalt);
        xtalts.insert(std::pair<uint64_t, uint8_t>(eui, xtalt));
    }

    ROS_INFO(" Building Anchor Tree...");
    for (auto it = seuis.begin(); it != seuis.end(); ++it)
    {
        if (m_anchorClocks.find(it->second) == m_anchorClocks.end())
        {
            std::map<uint64_t, ClockModel> m;
            m_anchorClocks.insert(std::make_pair(it->second, m));
        }
        if (it->first == it->second)
        {
            m_fastSyncMasterEUI = it->first;
        }
        else
        {
            ClockModel clock;
            m_anchorClocks[it->second].insert(std::make_pair(it->first, clock));
        }
    }

    for (auto it = m_anchorClocks.begin(); it != m_anchorClocks.end(); ++it)
    {
        ROS_INFO(" - Master %#llx", (unsigned long long)it->first);

        for (auto jt = it->second.begin(); jt != it->second.end(); ++jt)
        {
            ROS_INFO("  - Slave %#llx,", (unsigned long long)jt->first);
        }
    }

    ROS_INFO(" Clock Correction Pathfinder");
    for (auto it = seuis.begin(); it != seuis.end(); ++it)
    {
        bool found = false;
        uint64_t hop = it->first;
        std::vector<uint64_t> v;
        m_anchorPaths.insert(std::make_pair(it->first, v));

        // Repeat search multiple times
        for (int i = 0; i < m_anchorClocks.size(); ++i)
        {
            // Iterate through anchor sync map of slave maps
            for (auto jt = m_anchorClocks.begin(); jt != m_anchorClocks.end(); ++jt)
            {
                // If found in slave map, append to vector
                if (jt->second.find(hop) != jt->second.end())
                {
                    m_anchorPaths[it->first].push_back(jt->first);
                    hop = jt->first;
                    if (hop == m_fastSyncMasterEUI)
                    {
                        ROS_INFO(" - %#lx hops %lu", it->first, m_anchorPaths[it->first].size());
                        found = true;
                        break;
                    }
                }
            }
            if (found == true)
            {
                break;
            }
        }
    }

    ROS_INFO(" Initializing Trimming Publishers...");
    m_lastTrim = ros::Time::now();
    for (auto it = seuis.begin(); it != seuis.end(); ++it)
    {
        std::stringstream eui;
        eui << std::hex << it->first;
        m_pubTrim.insert(std::make_pair(it->first, n.advertise<atlas_msgs::Trim>("/atlas/anchor/" + eui.str() + "/trim", 10)));
    }
    for (auto it = xtalts.begin(); it != xtalts.end(); ++it)
    {
        m_lastTrimValues.insert(std::make_pair(it->first, it->second));
    }
}

void ClockCorrection::processSample(sample_t sample)
{
    //ROS_DEBUG(" Processing sample size %ld, seq %lld, txeui %#llx", (unsigned long)sample.meas.size(), (unsigned long long)sample.seq, (unsigned long long)sample.txeui);
    // Check if message is from designated sync anchor
    if(m_anchorClocks.find(sample.txeui) != m_anchorClocks.end())
    {
        //ROS_DEBUG(" Processing sync sample size %ld, seq %lld, txeui %#llx", (unsigned long)sample.meas.size(), (unsigned long long)sample.seq, (unsigned long long)sample.txeui);

        // Take reference timestamp from transmitting node
        double ref = (double)sample.meas[sample.txeui].ts / TICKS_PER_SECOND;;

        for (auto it = sample.meas.begin(); it != sample.meas.end(); ++it)
        {
            // Check if measurement is done by anchor in slave set (only then we need to use sync frame)
            if(m_anchorClocks[sample.txeui].find(it->first) != m_anchorClocks[sample.txeui].end())
            {
                // Process synchronization frame at the receiving slave sync anchor
                double toa = (double)it->second.ts / TICKS_PER_SECOND;

                //ROS_DEBUG(" - %#lx toa %f, ref %f", it->first, toa, ref);
                m_anchorClocks[sample.txeui][it->first].processSynchronizationFrame(toa, ref);
            }
        }
    }
    // Correct TOA message from positioning node
    else
    {
        ROS_INFO(" Processing sample size %lu, seq %lu, txeui %#lx", sample.meas.size(), sample.seq, sample.txeui);

        for (auto it = sample.meas.begin(); it != sample.meas.end();)
        {
            bool valid = true;
            double tof = 0.0;
            double toa = (double)it->second.ts / TICKS_PER_SECOND;
            uint64_t eui = it->first;

            // Iterate through clock sync path
            int i = 0;
            for (auto jt = m_anchorPaths[it->first].begin(); jt != m_anchorPaths[it->first].end(); ++jt)
            {
                ros::Duration d = sample.hts - m_anchorClocks[*jt][eui].getLastUpdate();
                ros::Duration t(SYNC_TIMEOUT);
                if(d > t) //ToDo: Implement while taking care of SFTAG case
                {
                    ROS_WARN("Last sync older than t: %fms, d: %fms, txeui: %#lx, hopeui: %#lx, rxeui: %#lx", t.toSec(), d.toSec(), *jt, eui, it->first);
                    valid = false;
                }

                double ntoa = m_anchorClocks[*jt][eui].getCorrectedTOA(toa);
                //ROS_INFO("Sync step %d, ToA %fm, NToA %fm, txeui: %#lx, hopeui: %#lx, rxeui: %#lx", i, toa*299792458, ntoa*299792458, *jt, eui, it->first);
                toa = ntoa;

                tof += sqrt((m_anchorPositions[*jt] - m_anchorPositions[eui]).array().square().sum()) / 299792458.0;
                eui = *jt;
                ++i;
            }

            //ROS_INFO(" ToA %fm, ToF %fm", toa*299792458, tof*299792458);

            it->second.toa = toa + tof;
            //it->second.toa = toa - tof;

            if(!valid)
            {
                ROS_WARN("Last sync too old, discarding...");
                sample.meas.erase(it++);
            }
            else
            {
                ++it;
            }
        }

        // Extracting TDOA from TOAs
        int row = 0;
        double firstTOA = 0.0;
        for (auto it = sample.meas.begin(); it != sample.meas.end(); ++it)
        {
            if(row == 0)
            {
                firstTOA = it->second.toa;
            }
            it->second.toa = (it->second.toa - firstTOA) * 299792458.0;
            row++;
        }

        m_samples.push_back(sample);
    }


    // Check if need to trim:
    ros::Duration d = ros::Time::now() - m_lastTrim;
    ros::Duration t(TRIM_INTERVAL);
    if(d > t)
    {
        // ToDo: Trim sync master to the mean of all anchors

        ROS_INFO("Re-Trimming after %fs", d.toSec());

        for (auto it = m_pubTrim.begin(); it != m_pubTrim.end(); ++it)
        {
            double drift = 0.0;
            uint64_t eui = it->first;

            // Iterate through clock sync path
            for (auto jt = m_anchorPaths[it->first].begin(); jt != m_anchorPaths[it->first].end(); ++jt)
            {
                drift += m_anchorClocks[*jt][eui].getLastDrift();
                eui = *jt;
            }

            uint8_t last = m_lastTrimValues[it->first];

            // Adjust range of value for trim register
            double driffOffset = drift * 1e6 * 0.1; // Assume p control p=0.1
            double driftTemp = -driffOffset + last;
            if (driftTemp > 0x20)
            {
                driftTemp = 0x20;
            }
            else if (driftTemp < 0x00)
            {
                driftTemp = 0x00;
            }
            uint8_t xtalt = std::round(driftTemp);

            ROS_INFO(" - %#lx drift: %fus/s, last: %#x, xtalt: %#x", it->first, drift * 1e6, last, xtalt);

            int8_t diff = xtalt - last;

            // Only send if changed
            if(abs(diff) > 2)
            {
                // ToDo: Send trim message
                //ROS_INFO(" - Adjusting");
                //atlas_msgs::Trim msg;
                //msg.xtalt = xtalt;
                //it->second.publish(msg);

                //m_lastTrimValues[it->first] = xtalt;
            }
        }

        m_lastTrim = ros::Time::now();
    }
}

void ClockCorrection::processSamples(std::vector<sample_t> *samples)
{
    for (auto it = samples->begin(); it != samples->end(); ++it)
    {
        processSample(*it);
    }
}

std::vector<sample_t> ClockCorrection::getCorrectedSamples()
{
    std::vector<sample_t> s = m_samples;
    m_samples.clear();
    return s;
}
