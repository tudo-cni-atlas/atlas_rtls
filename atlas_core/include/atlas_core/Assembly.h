//
//  Assembly.h
//  atlas core
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__assembly__
#define __atlas__assembly__

#include <stdio.h>
#include <string>
#include <vector>
#include <map>

#include "ros/ros.h"
#include <atlas_msgs/Toa.h>
#include <atlas_msgs/ToaLde.h>
#include <atlas_msgs/ToaImu.h>
#include <atlas_msgs/ToaImuLde.h>

#include "atlas_core/atlas_types.h"


class Assembly
{
private:
    const uint64_t ticksPerRevolution = 0x10000000000;

    std::map<uint64_t, std::map<uint64_t, std::map<uint64_t, measurement_t> > > m_measurements;
    std::map<uint64_t, uint64_t> m_lastSequence;
    std::map<uint64_t, uint64_t> m_lastTimestamp;

    void toaCallback(const atlas_msgs::Toa& msg);
    void toaLdeCallback(const atlas_msgs::ToaLde& msg);
    void toaImuCallback(const atlas_msgs::ToaImu& msg);
    void toaImuLdeCallback(const atlas_msgs::ToaImuLde& msg);

    std::map<uint64_t, ros::Subscriber> m_subToa;
    std::map<uint64_t, ros::Subscriber> m_subToaLde;
    std::map<uint64_t, ros::Subscriber> m_subToaImu;
    std::map<uint64_t, ros::Subscriber> m_subToaImuLde;

public:
    Assembly ();
    ~Assembly();

    void initialize(ros::NodeHandle n);
    void extractSamples(std::vector<sample_t> *s);
};

#endif /* defined(__atlas__assembly__) */
