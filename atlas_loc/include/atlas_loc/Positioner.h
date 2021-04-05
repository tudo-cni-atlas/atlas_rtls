//
//  Positioner.h
//  atlas
//
//  Created by Janis on 18.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__positioner__
#define __atlas__positioner__

#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <deque>

#include <atlas_msgs/Sample.h>
#include <atlas_msgs/Measurement.h>

#include "atlas_loc/atlas_types.h"

typedef struct
{
    ros::Time lastUpdate;
    ros::Time initialTs;
    Eigen::VectorXd state;
    Eigen::MatrixXd stateCovariance;
    Eigen::MatrixXd W;
    Eigen::MatrixXd V;
    bool imu;
    bool sqd;
    bool constrained;
} ekf_t;

class PositionerTDOA
{
private:
    // system parameters
    std::map<uint64_t, Eigen::Vector3d> m_anchorPositions;

    // validation
    std::map<uint64_t, position_t> m_lastPosition;
    std::map<uint64_t, sample_t> m_lastSample;
    std::map<uint64_t, uint64_t> m_updateCount;

    // ekf initialization and state
    std::map<uint64_t, ekf_t> m_ekf;
    bool m_imu;
    bool m_sqd;
    bool m_constrained;
    int m_minAnchor = 6;
    double m_minX, m_maxX, m_minY, m_maxY, m_minZ, m_maxZ;
    int m_dimensions;
    double m_processNoise;
    double m_measurementNoise;
    double m_initialStateVariance;
    double m_initialStateVarianceDelta;
    double m_initialInterval;
    Eigen::VectorXd m_initialState;

    //leveling in 2d mode
    std::vector<double> m_fixedZ;
    std::vector<int> m_cellLevels;
    std::map<uint64_t, int> m_anchorLevels;
    std::map<uint64_t, int> m_tagLevel;
    std::map<uint64_t, int> m_prevTagLevel;

    // outlier detection
    double m_outlierThreshold;
    double m_outlierThresholdDelta;

    // orientation offset
    Eigen::Vector3d m_orientOffset;
    
    // sample collection
    std::deque<sample_t> m_samples;
    std::map<uint64_t, ros::Subscriber> m_subSample;
    void sampleCallback(const atlas_msgs::Sample& msg);
    
    //Predictive Zone Selection
    bool m_pzs;
    std::vector<std::vector<uint64_t>> m_cellAnchors;
    std::map<uint64_t, int> m_anchorCells;
    Eigen::MatrixXd m_cellBounds;
    Eigen::MatrixXd m_cellCenters;
    int m_cellsPerZone;

    std::map<uint64_t, int> m_tagCell;
    std::map<uint64_t, std::vector<uint64_t>> m_anchorsInZone;

public:
    PositionerTDOA ();

    void initialize(ros::NodeHandle n);
    void extractSamples(std::vector<sample_t> *samples);

    void createNewEKF(uint64_t eui, ros::Time ts);
    bool calculatePositionEKFInner(const sample_t &s, position_t *p);
    bool calculatePositionEKFConstraints(const sample_t &s, position_t *p);
    bool calculatePositionEKF(const sample_t &s, position_t *p);

    //Predictive Zone Selection
    bool calculatePositionEKFInnerZoning(const sample_t &s, position_t *p, int count);
    bool calculatePositionEKFZoning(const sample_t &s, position_t *p);
};

#endif /* defined(__atlas__tdoa__) */