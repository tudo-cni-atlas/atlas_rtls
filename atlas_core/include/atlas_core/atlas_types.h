//
//  atlas_types.h
//  atlas core
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__atlas_types__
#define __atlas__atlas_types__

#include <stdio.h>
#include <string>
#include <vector>
#include <map>
#include <set>

#include <eigen3/Eigen/Dense>
#include "ros/ros.h"


typedef struct
{
    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;
    Eigen::Vector3d magn;
    Eigen::Vector4d quat;
} imu_t;

typedef struct
{
    uint16_t    maxNoise;
    uint16_t    firstPathAmp1;
    uint16_t    stdNoise;
    uint16_t    firstPathAmp2;
    uint16_t    firstPathAmp3;
    uint16_t    maxGrowthCIR;
    uint16_t    rxPreamCount;
    uint16_t    firstPath;
} lde_t;

typedef struct
{
    ros::Time hts;
    uint64_t ts;
    uint64_t seq;
    double toa;
    imu_t imu;
    lde_t lde;
} measurement_t;

typedef struct
{
    ros::Time hts;
    uint64_t txeui;
    uint64_t seq;
    std::map<uint64_t, measurement_t> meas;
    imu_t imu;
} sample_t;

#endif /* defined(__atlas__atlas_types__) */