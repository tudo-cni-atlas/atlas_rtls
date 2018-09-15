//
//  Reporter.h
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__reporter__
#define __atlas__reporter__

#include <stdio.h>
#include <string>
#include <map>

#include "ros/ros.h"
#include "atlas_loc/atlas_types.h"


class Reporter
{
private:
    ros::NodeHandle m_n;

    std::map<uint64_t, uint32_t> m_seq;
    std::map<uint64_t, ros::Publisher> m_samplePubs;
    std::map<uint64_t, ros::Publisher> m_pointPubs;

public:
    Reporter ();
    ~Reporter();
    void initialize(ros::NodeHandle n);
    void reportSample(const sample_t &s);
    void reportPosition(const position_t &p);
};


#endif /* defined(__atlas__reporter__) */