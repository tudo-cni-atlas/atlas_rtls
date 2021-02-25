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
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <cmath>
#include <queue>
#include <numeric>

#include "atlas_core/atlas_types.h"
#include "atlas_core/ClockCorrection.h"

#include <atlas_msgs/Trim.h>

typedef std::pair< double, int > pdi;
typedef std::pair< int, double > pid;


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

double ClockModel::getVarianceDev(double toa, double ref)
{
    //compute sync error
    double ctoa = toa - m_lastOffset - m_lastDrift*(toa-m_lastSync);
    double dev = ctoa - ref;

    if(dev != 0)
    {
        m_que.push_back(dev);
    }

    //get variance of defined sample size
    double sum = std::accumulate(std::begin(m_que), std::end(m_que), 0.0);
    double mean = sum / m_que.size();
    double accum = 0; 

    std::for_each(std::begin(m_que), std::end(m_que), [&](const double d){
        accum += (d-mean)*(d-mean);
    });

    double varDev = accum / (m_que.size()-1);

    if (m_que.size() == SAMPLE_SIZE){
        m_que.pop_front();
    }

    //return variance
    if(varDev > 0)
    {
      return varDev;
    }
    else
    {
       double k = 1.0;
       return k;
    }    
}

void ClockCorrection::findBestPath(int start){
    //create adjacency list of sync graph
    std::vector<pid> adj[100];

    int count = sqrt(m_syncGraph.size());

    for(int i = 0; i < count; i++)
    {
        for(int j = 0; j < count; j++)
        {
            if(i!=j)
            {
                adj[i].push_back(pid(j, m_syncGraph(i,j)));
                //m_syncGraph(i,j) = 1.0;
            }
            else
            {
                adj[i].push_back(pid(j, 0));
            }
        }
    }

    //compute shortest distance to defined reference sync master (Dijkstra's algorithm)
    int numVert = seuis.size();
    double d[numVert];
    int parent[numVert];

    std::priority_queue<pdi, std::vector<pdi>, std::greater <pdi> > Q;

    for (int i = 0; i < numVert; i++)
    {
        d[i] = 1.0;
        parent[i] = -1;
    }

    Q.push(pdi(0, start));
    d[start] = 0;

    while(!Q.empty()) 
    {
        int u = Q.top().second;
        double c = Q.top().first;
        Q.pop();

        if(d[u] < c) 
        {
            continue;
        }

        for(int i=0; i < adj[u].size(); i++)
        {
            int v = adj[u][i].first;
            double w = adj[u][i].second;

            if(d[v] > d[u] + w) 
            {
                d[v] = d[u] + w;
                parent[v] = u;
                Q.push(pdi(d[v], v));
            }
        }
    }

    //get vertices of shortest sync path for each anchor to its respective reference sync anchor & update paths
    for (int i = 0; i < numVert; i++)
    {
        if(std::find(m_masterSlaveAssign[m_verticeEui[start]].begin(), m_masterSlaveAssign[m_verticeEui[start]].end(), 
                    m_verticeEui[i])!= m_masterSlaveAssign[m_verticeEui[start]].end())
        {
            //std::cout << std::hex << m_verticeEui[i] << "[.................]" << std::setprecision (std::numeric_limits<long double>::digits10 + 1) << d[i] << std::endl;
            std::vector<uint64_t> path;
            getBestPath(parent, i, path);

            if(i != start)
            {
                path.erase(path.begin());
                m_anchorPaths.at(m_verticeEui[i]) = path;
            }
        }
    }
}

void ClockCorrection::getBestPath(int parent[], int j, std::vector<uint64_t> &path)
{
    if (parent[j] == -1)
    {
        path.push_back(m_verticeEui[j]);
        return;
    }
    
    path.push_back(m_verticeEui[j]);
    getBestPath(parent, parent[j], path);
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

    //std::map<uint64_t, uint64_t> seuis;
    std::map<uint64_t, uint8_t> xtalts;

    XmlRpc::XmlRpcValue anchors;
    n.getParam("/atlas/anchor", anchors);
    ROS_ASSERT(anchors.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    double count;

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

        count++;
    }

    ROS_INFO(" Building Anchor Tree...");
    for (auto it = seuis.begin(); it != seuis.end(); ++it)
    {
        m_verticeEui.push_back(it->first);

        if (m_anchorClocks.find(it->second) == m_anchorClocks.end())
        {
            std::map<uint64_t, ClockModel> m;
            m_anchorClocks.insert(std::make_pair(it->second, m));
        }
        if (it->first == it->second)
        {
            //get reference sync masters
            m_fastSyncMasterEUI.push_back(it->first);
            auto itr = std::find(m_verticeEui.begin(), m_verticeEui.end(), it->first);
            m_syncMasterVertice = std::distance(m_verticeEui.begin(), itr);
            std::cout << "Tree root: " << std::hex << it->first << "..."<< std::endl;

            //assign each anchor to its respective reference sync master
            std::vector<uint64_t> v;
            m_masterSlaveAssign.insert(std::make_pair(it->first, v));
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
    m_lastPathUpdate = ros::Time::now();
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
                    if (std::find(m_fastSyncMasterEUI.begin(), m_fastSyncMasterEUI.end(), hop) != m_fastSyncMasterEUI.end())
                    {
                        m_masterSlaveAssign[jt->first].push_back(it->first);
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

    n.getParam("/atlas/dbld", m_dbld);

    if(m_dbld == true){
        ROS_INFO("Best Link Discovery Initialized");
        m_pathSet=false;
        m_startTimeSet=false;

        //initialize sync graph
        m_syncGraph.setOnes(count, count);
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
    if(m_startTimeSet == false)
    {
        m_startTime = ros::Time::now();
        m_startTimeSet = true;
    }

    //ROS_DEBUG(" Processing sample size %ld, seq %lld, txeui %#llx", (unsigned long)sample.meas.size(), (unsigned long long)sample.seq, (unsigned long long)sample.txeui);
    // Check if message is from designated sync anchor
    if(m_anchorClocks.find(sample.txeui) != m_anchorClocks.end())
    {
        //ROS_DEBUG(" Processing sync sample size %ld, seq %lld, txeui %#llx", (unsigned long)sample.meas.size(), (unsigned long long)sample.seq, (unsigned long long)sample.txeui);

        // Take reference timestamp from transmitting node
        double ref = (double)sample.meas[sample.txeui].ts / TICKS_PER_SECOND;;

        for (auto it = sample.meas.begin(); it != sample.meas.end(); ++it)
        {
            if(m_dbld==true)
            {
                double toa = (double)it->second.ts / TICKS_PER_SECOND;
                double varDev = m_anchorClocks[sample.txeui][it->first].getVarianceDev(toa,ref);

                ROS_DEBUG(" - %#lx toa %f, ref %f", it->first, toa, ref);

                m_anchorClocks[sample.txeui][it->first].processSynchronizationFrame(toa, ref);

                //create sync graph
                auto itr_i = std::find(m_verticeEui.begin(), m_verticeEui.end(), sample.txeui);
                int idx_i = std::distance(m_verticeEui.begin(), itr_i);
                auto itr_j = std::find(m_verticeEui.begin(), m_verticeEui.end(), it->first);
                int idx_j = std::distance(m_verticeEui.begin(), itr_j);
                
                    if(idx_i!=idx_j)
                    {
                        m_syncGraph(idx_i, idx_j) = varDev;
                    }
                    else
                    {
                        m_syncGraph(idx_i, idx_j) = 0;
                    }
            }
            else
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
    }
    // Correct TOA message from positioning node
    else
    {
        //ROS_INFO(" Processing sample size %lu, seq %lu, txeui %#lx", sample.meas.size(), sample.seq, sample.txeui);
        if(m_dbld==true)
        {
            //ros::Duration dp = ros::Time::now() - m_lastPathUpdate;
            ros::Duration ds = ros::Time::now() - m_startTime;
            //ros::Duration tp(UPDATE_PATH_INTERVAL);
            ros::Duration ts(START_UPDATE_PATH);

            if((ds > ts) && (m_pathSet==false))
            {
                for(int i = 0; i < m_fastSyncMasterEUI.size(); i++){
                    auto itr = std::find(m_verticeEui.begin(), m_verticeEui.end(), m_fastSyncMasterEUI[i]);
                    m_syncMasterVertice = std::distance(m_verticeEui.begin(), itr);
                    findBestPath(m_syncMasterVertice);
                }

                m_lastPathUpdate = ros::Time::now();

                for (auto it = seuis.begin(); it != seuis.end(); ++it){
                    ROS_INFO(" - %#lx hops %lu", it->first, m_anchorPaths[it->first].size());
                    //ROS_INFO(" - %#lx clockcount %lu", it->first, m_anchorClocks[it->second].size());
                    for (int i = 0; i<m_anchorPaths[it->first].size(); i++){
                        std::cout << m_anchorPaths[it->first][i] << std::endl;
                    }
                }
                m_pathSet=true;
            }
        }

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
        //double firstTOA = 0.0;

        std::map<uint64_t, double> firstTOA; //firstTOA for specific tree root

        for (auto it = sample.meas.begin(); it != sample.meas.end(); ++it)
        {
            uint64_t tree_root;

            if(m_anchorPaths[it->first].size()==0)
            {
                tree_root = it->first;
            }
            else
            {
                tree_root = m_anchorPaths[it->first].back();
            }

            if(firstTOA.find(tree_root) == firstTOA.end())
            {
                firstTOA.insert(std::make_pair(tree_root, it->second.toa));
            }

            it->second.toa = (it->second.toa - firstTOA.at(tree_root)) * 299792458.0;
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
