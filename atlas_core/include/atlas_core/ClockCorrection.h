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

#include "atlas_core/atlas_types.h"

#include "ros/ros.h"

class ClockModel
{
private:
    double m_lastSync;
    double m_lastOffset;
    double m_lastDrift;
    ros::Time m_lastUpdate;

public:
    ClockModel();
    ~ClockModel();

    void processSynchronizationFrame(double toa, double ref);
    double getCorrectedTOA(double toa) const;
    double getLastOffset() const;
    double getLastDrift() const;
    ros::Time getLastUpdate() const;
};

class ClockCorrection
{
private:
    // anchor clock correction
    const uint64_t TICKS_PER_SECOND = 128 * 499.2e6;
    const double SYNC_TIMEOUT = 0.4;
    uint64_t m_fastSyncMasterEUI;
    std::map<uint64_t, std::map<uint64_t, ClockModel> > m_anchorClocks;
    std::map<uint64_t, std::vector<uint64_t> > m_anchorPaths;
    std::map<uint64_t, Eigen::Vector3d> m_anchorPositions;
    std::vector<sample_t> m_samples;

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
};


#endif /* defined(__atlas__clockcorrection__) */
