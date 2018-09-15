//
//  parser.h
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__parser_tdoa__
#define __atlas__parser_tdoa__

#include <stdio.h>
#include <string>
#include <map>

#include "ros/ros.h"

#include "protocol.h"
#include <atlas_msgs/Trim.h>
#include <atlas_msgs/AssociationResponse.h>


class Parser
{
private:
    ros::NodeHandle m_n;

    ros::Publisher m_toaPub;
    ros::Publisher m_toaLdePub;
    ros::Publisher m_toaImuPub;
    ros::Publisher m_toaImuLdePub;

    ros::Subscriber m_trimSub;
    ros::Subscriber m_subAssociationResponse;


    const uint64_t ticksPerRevolution = 0x10000000000;

    protocol::Protocol* m_decoder;
    std::string m_name;
    std::map<uint64_t, uint64_t> m_lastSequence;
    uint64_t m_lastTimestamp;

    void trimCallback(const atlas_msgs::Trim& msg);
    void associationResponseCallback(const atlas_msgs::AssociationResponse& msg);


public:
    Parser ();
    ~Parser();

    void initialize(ros::NodeHandle *n, const uint64_t eui);
    void poll();
};

#endif /* defined(__atlas__parser_tdoa__) */
