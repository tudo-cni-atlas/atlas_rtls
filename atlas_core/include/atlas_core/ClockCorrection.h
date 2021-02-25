//
//  ClockCorrection.h
//  atlas core
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__clockcorrection__
#define __atlas__clockcorrection__

#include <stdio.h>
#include <vector>
#include <map>
#include <deque>

#include "atlas_core/atlas_types.h"

#include "ros/ros.h"

class ClockModel
{
private:
    double m_lastSync;
    double m_lastOffset;
    double m_lastDrift;
    ros::Time m_lastUpdate;

    //getVariance
    std::deque<double> m_que;
    ros::Time m_lastVarUpdate;
    const int SAMPLE_SIZE = 160;

public:
    ClockModel();
    ~ClockModel();

    void processSynchronizationFrame(double toa, double ref);
    double getCorrectedTOA(double toa) const;
    double getLastOffset() const;
    double getLastDrift() const;
    ros::Time getLastUpdate() const;

    // Dynamic Best Link Discovery
    double getVarianceDev(double toa, double ref);
};

class ClockCorrection
{
private:
    // anchor clock correction
    const uint64_t TICKS_PER_SECOND = 128 * 499.2e6;
    const double SYNC_TIMEOUT = 0.4;
    std::vector<uint64_t> m_fastSyncMasterEUI;
    std::map<uint64_t, std::vector<uint64_t> > m_masterSlaveAssign;
    std::map<uint64_t, std::map<uint64_t, ClockModel> > m_anchorClocks;
    std::map<uint64_t, std::vector<uint64_t> > m_anchorPaths;
    std::map<uint64_t, Eigen::Vector3d> m_anchorPositions;
    std::vector<sample_t> m_samples;

    std::map<uint64_t, uint64_t> seuis;

    // Dynamic Best Link Discovery
    bool m_dbld;
    double m_varDev;
    int m_syncMasterVertice;
    std::vector<uint64_t> m_verticeEui;
    const double START_UPDATE_PATH = 15;
    ros::Time m_lastPathUpdate;
    Eigen::MatrixXd m_syncGraph;
    bool m_pathSet;
    bool m_startTimeSet;
    ros::Time m_startTime;

    // trimming
    const double TRIM_INTERVAL = 10.0;
    ros::Time m_lastTrim;
    std::map<uint64_t, uint8_t> m_lastTrimValues;
    std::map<uint64_t, ros::Publisher> m_pubTrim;

public:
    ClockCorrection();
    ~ClockCorrection();

    void initialize(ros::NodeHandle n);
    void processSample(sample_t sample);
    void processSamples(std::vector<sample_t> *samples);
    std::vector<sample_t> getCorrectedSamples();

    void findBestPath(int txvert);
    void getBestPath(int parent[], int j, std::vector<uint64_t> &path);
};


#endif /* defined(__atlas__clockcorrection__) */
