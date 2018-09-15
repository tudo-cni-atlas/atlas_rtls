//
//  parser.cpp
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#include "parser.h"

#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <map>
#include <string>

#include "protocol.h"
#include "protocol_structures.h"
#include "protocol_commands.cpp"

#include <atlas_msgs/Toa.h>
#include <atlas_msgs/ToaLde.h>
#include <atlas_msgs/ToaImu.h>
#include <atlas_msgs/ToaImuLde.h>
#include <atlas_msgs/Trim.h>

#include <std_msgs/UInt32.h>

#include <math.h>
#include <protocol_structures.h>

#include "ros/ros.h"

typedef enum
{
    CFG_MODE_NO_MODE = 0,
    CFG_MODE_TDOA_SYNC,
    CFG_MODE_TDOA_ANCHOR,
    CFG_MODE_TDOA_SANCHOR,
    CFG_MODE_TDOA_RTAG,
    CFG_MODE_TDOA_STAG,
    CFG_MODE_NUM_MODES
} cfgMode_t;

typedef enum reportingModes
{
    NO_REP,
    REP_STD,                       // Basic
    REP_STD_LDE,                   // With LDE
    REP_STD_DIAG,                  // With LDE and DIAG
    NUM_REP_MODES,
} repMode_t;


Parser::Parser()
{

}

Parser::~Parser()
{

}

void Parser::initialize(ros::NodeHandle *n, const uint64_t eui)
{
    m_n = *n;

    std::stringstream seui;
    seui << std::hex << eui;

    std::stringstream name_stream;
    name_stream << "/atlas/anchor/";
    name_stream << seui.str();
    m_name = name_stream.str();

    std::cout << "Anchor " + m_name + " Configuration:" << std::endl;

    std::string port;
    if (m_n.getParam(m_name + "/port", port))
    {
        ROS_INFO("PARAM port: %s", port.c_str());
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'port'");
    }

    int mode;
    if (m_n.getParam(m_name + "/mode", mode))
    {
        ROS_INFO("PARAM mode: %d", mode);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'mode'");
    }

    std::string tdoaSyncEui;
    if (m_n.getParam(m_name + "/tdoaSyncEui", tdoaSyncEui))
    {
        ROS_INFO("PARAM tdoaSyncEui: %s", tdoaSyncEui.c_str());
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'tdoaSyncEui'");
    }
    uint64_t masterEui = std::stoull(tdoaSyncEui, nullptr, 16);

    int tdoaSyncOffset;
    if (m_n.getParam(m_name + "/tdoaSyncOffset", tdoaSyncOffset))
    {
        ROS_INFO("PARAM tdoaSyncOffset: %d", tdoaSyncOffset);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'tdoaSyncOffset'");
    }

    int randomAccessSlots;
    if (mode == 3 && m_n.getParam(m_name + "/randomAccessSlots", randomAccessSlots)) // If the module is a sync anchor, get its default random access slots
    {
        ROS_INFO("PARAM randomAccessSlots: %d", randomAccessSlots);
    }
    else if (mode == 3)
    {
        ROS_ERROR("PARAM FAILED to get 'randomAccessSlots'");
    }

    int channelConfig;
    if (m_n.getParam("/atlas/channelConfig", channelConfig))
    {
        ROS_INFO("PARAM channelConfig: %d", channelConfig);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'channelConfig'");
    }

    int tdoaSlotDuration;
    if (m_n.getParam("/atlas/tdoa/slotDuration", tdoaSlotDuration))
    {
        ROS_INFO("PARAM tdoaSlotDuration: %d", tdoaSlotDuration);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'tdoaSlotDuration'");
    }

    int tdoaSyncPeriod;
    if (m_n.getParam("/atlas/tdoa/syncPeriod", tdoaSyncPeriod))
    {
        ROS_INFO("PARAM tdoaSyncPeriod: %d", tdoaSyncPeriod);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'tdoaSyncPeriod'");
    }

    m_decoder = new protocol::Protocol(port, m_name);

    ros::Duration(0.01).sleep();

    protocol::msgSetReporting_t msg;
    msg.reporting = NO_REP;
    m_decoder->sendMsgPush(msg);
    ROS_INFO("-> %s Deactivate Reporting", m_name.c_str());

    ros::Duration(0.01).sleep();
    m_decoder->flush();

    protocol::msgConfig_t config;
    m_decoder->sendMsgPoll(&config);
    ROS_INFO("<- %s Current eui:%#llx, mode:%d, channel:%d", m_name.c_str(), (unsigned long long)config.eui, config.mode, config.channel);

    config.eui = eui;
    config.mode = mode;
    config.channel = channelConfig;
    m_decoder->sendMsg(config);
    ROS_INFO("-> %s New eui:%#llx, mode:%d, channel:%d", m_name.c_str(), (unsigned long long)config.eui, config.mode, config.channel);

    ros::Duration(0.01).sleep();

    protocol::msgSetClockTrim_t trim;
    m_decoder->sendMsgPoll(&trim);
    ROS_INFO("<- %s Current xtalt:%#x", m_name.c_str(), trim.clockTrim);
    m_n.setParam(m_name + "/xtalt", trim.clockTrim);

    ros::Duration(0.01).sleep();

    if(mode == CFG_MODE_TDOA_SYNC)
    {
        protocol::msgConfigSync_t msg;
        m_decoder->sendMsgPoll(&msg);
        ROS_INFO("<- %s Current slotDuration:%#x, syncPeriod:%#x", m_name.c_str(), msg.slotDuration, msg.syncPeriod);

        msg.slotDuration = tdoaSlotDuration;
        msg.syncPeriod = tdoaSyncPeriod;
        m_decoder->sendMsg(msg);
        ROS_INFO("-> %s New slotDuration:%#x, syncPeriod:%#x", m_name.c_str(), msg.slotDuration, msg.syncPeriod);
    }
    else if(mode == CFG_MODE_TDOA_ANCHOR)
    {
        // No further config required...
    }
    else if(mode == CFG_MODE_TDOA_SANCHOR)
    {
        protocol::msgConfigSyncAnchor_t msg;
        m_decoder->sendMsgPoll(&msg);
        ROS_INFO("<- %s Current slotDuration:%#x, syncPeriod:%#x, masterEui:%#llx, masterOffset:%#x, ", m_name.c_str(), msg.slotDuration, msg.syncPeriod, (unsigned long long)msg.masterEui, msg.masterOffset);

        msg.slotDuration = tdoaSlotDuration;
        msg.syncPeriod = tdoaSyncPeriod;
        msg.masterEui = masterEui;
        msg.masterOffset = tdoaSyncOffset;
        msg.randomAccessSlots = randomAccessSlots;
        m_decoder->sendMsg(msg);
        ROS_INFO("-> %s New slotDuration:%#x, syncPeriod:%#x, masterEui:%#llx, masterOffset:%#x, ", m_name.c_str(), msg.slotDuration, msg.syncPeriod, (unsigned long long)msg.masterEui, msg.masterOffset);
    }

    msg.reporting = REP_STD;
    //msg.reporting = REP_STD_LDE;

    m_decoder->sendMsgPush(msg);
    ROS_INFO("-> %s Activate Reporting", m_name.c_str());

    m_toaPub = m_n.advertise<atlas_msgs::Toa>("/atlas/anchor/" + seui.str() + "/toa", 100);
    m_toaLdePub = m_n.advertise<atlas_msgs::ToaLde>("/atlas/anchor/" + seui.str() + "/toaLde", 100);
    m_toaImuPub = m_n.advertise<atlas_msgs::ToaImu>("/atlas/anchor/" + seui.str() + "/toaImu", 100);
    m_toaImuLdePub = m_n.advertise<atlas_msgs::ToaImuLde>("/atlas/anchor/" + seui.str() + "/toaImuLde", 100);

    m_trimSub = m_n.subscribe("/atlas/anchor/" + seui.str() + "/trim", 10, &Parser::trimCallback, this);

    if (mode == 3)  // If the module is a sync anchor
        m_subAssociationResponse = m_n.subscribe("/atlas/anchor/" + seui.str() + "/associationResponse", 12, &Parser::associationResponseCallback, this, ros::TransportHints().tcpNoDelay());


    m_lastTimestamp = 0;
}

void Parser::poll()
{
    protocol::packet_t packet;
    if(m_decoder->poll(&packet))
    {
        if (m_decoder->calculateCheckSum(&packet) != packet.checksum)
        {
            ROS_ERROR("!! %s Checksum x%#x, x%#x, x%#x, x%#x", m_name.c_str(), packet.messageId, packet.payloadLength, packet.checksum, m_decoder->calculateCheckSum(&packet));
            for(uint16_t i = 0; i < packet.payloadLength; ++i)
            {
                ROS_ERROR(" payload %d x%#x", i, (int)packet.payload[i]);
            }
        }
        else
        {
            if(packet.messageId == msg_id_toa)
            {
                protocol::msgToa_t msg;
                memcpy(&msg, packet.payload, sizeof(protocol::msgToa_t));
                uint64_t ts = msg.rxTs;
                uint64_t rxeui = msg.rxId;

                if(m_lastTimestamp == 0)
                {
                    m_lastTimestamp = ts;
                }
                else
                {
                    int64_t lastts = m_lastTimestamp;
                    int64_t res = lastts % ticksPerRevolution;

                    if((int64_t)ts - (int64_t)res > 0)
                    {
                        m_lastTimestamp = lastts + (ts - res);
                        //std::cout << "+ ts:" << ts << " lastts:" << lastts << " res:" << res << std::endl;
                    }
                    if((int64_t)ts - (int64_t)res < 0)
                    {
                        m_lastTimestamp = lastts + (ts - res) + ticksPerRevolution;
                        //std::cout << "- ts:" << ts << " lastts:" << lastts << " res:" << res << std::endl;
                    }
                }

                atlas_msgs::Toa toa_msg;
                toa_msg.ts = m_lastTimestamp;
                toa_msg.txId = msg.txId;
                toa_msg.rxId = msg.rxId;
                toa_msg.seq = msg.seqNr;

                m_toaPub.publish(toa_msg);
            }
            else if(packet.messageId == msg_id_toa_lde)
            {
                protocol::msgToaLde_t msg;
                memcpy(&msg, packet.payload, sizeof(protocol::msgToaLde_t));
                uint64_t ts = msg.rxTs;
                uint64_t rxeui = msg.rxId;

                if(m_lastTimestamp == 0)
                {
                    m_lastTimestamp = ts;
                }
                else
                {
                    int64_t lastts = m_lastTimestamp;
                    int64_t res = lastts % ticksPerRevolution;

                    if((int64_t)ts - (int64_t)res > 0)
                    {
                        m_lastTimestamp = lastts + (ts - res);
                        //std::cout << "+ ts:" << ts << " lastts:" << lastts << " res:" << res << std::endl;
                    }
                    if((int64_t)ts - (int64_t)res < 0)
                    {
                        m_lastTimestamp = lastts + (ts - res) + ticksPerRevolution;
                        //std::cout << "- ts:" << ts << " lastts:" << lastts << " res:" << res << std::endl;
                    }
                }

                atlas_msgs::ToaLde toa;
                toa.ts = m_lastTimestamp;
                toa.txId = msg.txId;
                toa.rxId = msg.rxId;
                toa.seq = msg.seqNr;
                toa.lde.maxNoise = msg.maxNoise;
                toa.lde.firstPathAmp1 = msg.firstPathAmp1;
                toa.lde.stdNoise = msg.stdNoise;
                toa.lde.firstPathAmp2 = msg.firstPathAmp2;
                toa.lde.firstPathAmp3 = msg.firstPathAmp3;
                toa.lde.maxGrowthCIR = msg.maxGrowthCIR;
                toa.lde.rxPreamCount = msg.rxPreamCount;
                toa.lde.firstPath = msg.firstPath;

                m_toaLdePub.publish(toa);
            }
            else if(packet.messageId == msg_id_toa_imu)
            {
                protocol::msgToaImu_t msg;
                memcpy(&msg, packet.payload, sizeof(protocol::msgToaImu_t));
                uint64_t ts = msg.rxTs;
                uint64_t rxeui = msg.rxId;

                if(m_lastTimestamp == 0)
                {
                    m_lastTimestamp = ts;
                }
                else
                {
                    int64_t lastts = m_lastTimestamp;
                    int64_t res = lastts % ticksPerRevolution;

                    if((int64_t)ts - (int64_t)res > 0)
                    {
                        m_lastTimestamp = lastts + (ts - res);
                        //std::cout << "+ ts:" << ts << " lastts:" << lastts << " res:" << res << std::endl;
                    }
                    if((int64_t)ts - (int64_t)res < 0)
                    {
                        m_lastTimestamp = lastts + (ts - res) + ticksPerRevolution;
                        //std::cout << "- ts:" << ts << " lastts:" << lastts << " res:" << res << std::endl;
                    }
                }


                atlas_msgs::ToaImu toa;
                toa.ts = m_lastTimestamp;
                toa.txId = msg.txId;
                toa.rxId = msg.rxId;
                toa.seq = msg.seqNr;
                toa.imu.accel[0] = msg.acc[0];
                toa.imu.accel[1] = msg.acc[1];
                toa.imu.accel[2] = msg.acc[2];
                toa.imu.gyro[0] = msg.gyr[0];
                toa.imu.gyro[1] = msg.gyr[1];
                toa.imu.gyro[2] = msg.gyr[2];
                toa.imu.magn[0] = msg.mag[0];
                toa.imu.magn[1] = msg.mag[1];
                toa.imu.magn[2] = msg.mag[2];

                m_toaImuPub.publish(toa);

            }
            else if(packet.messageId == msg_id_toa_imu_lde)
            {
                protocol::msgToaImuLde_t msg;
                memcpy(&msg, packet.payload, sizeof(protocol::msgToaImuLde_t));
                uint64_t ts = msg.rxTs;
                uint64_t rxeui = msg.rxId;

                if(m_lastTimestamp == 0)
                {
                    m_lastTimestamp = ts;
                }
                else
                {
                    int64_t lastts = m_lastTimestamp;
                    int64_t res = lastts % ticksPerRevolution;

                    if((int64_t)ts - (int64_t)res > 0)
                    {
                        m_lastTimestamp = lastts + (ts - res);
                        //std::cout << "+ ts:" << ts << " lastts:" << lastts << " res:" << res << std::endl;
                    }
                    if((int64_t)ts - (int64_t)res < 0)
                    {
                        m_lastTimestamp = lastts + (ts - res) + ticksPerRevolution;
                        //std::cout << "- ts:" << ts << " lastts:" << lastts << " res:" << res << std::endl;
                    }
                }

                atlas_msgs::ToaImuLde toa;
                toa.ts = m_lastTimestamp;
                toa.txId = msg.txId;
                toa.rxId = msg.rxId;
                toa.seq = msg.seqNr;
                toa.imu.accel[0] = msg.acc[0];
                toa.imu.accel[1] = msg.acc[1];
                toa.imu.accel[2] = msg.acc[2];
                toa.imu.gyro[0] = msg.gyr[0];
                toa.imu.gyro[1] = msg.gyr[1];
                toa.imu.gyro[2] = msg.gyr[2];
                toa.imu.magn[0] = msg.mag[0];
                toa.imu.magn[1] = msg.mag[1];
                toa.imu.magn[2] = msg.mag[2];
                toa.lde.maxNoise = msg.maxNoise;
                toa.lde.firstPathAmp1 = msg.firstPathAmp1;
                toa.lde.stdNoise = msg.stdNoise;
                toa.lde.firstPathAmp2 = msg.firstPathAmp2;
                toa.lde.firstPathAmp3 = msg.firstPathAmp3;
                toa.lde.maxGrowthCIR = msg.maxGrowthCIR;
                toa.lde.rxPreamCount = msg.rxPreamCount;
                toa.lde.firstPath = msg.firstPath;

                m_toaImuLdePub.publish(toa);
            }
        }
    }
}

void Parser::trimCallback(const atlas_msgs::Trim& msg)
{
    protocol::msgSetClockTrim_t trim;
    trim.clockTrim = msg.xtalt;
    m_decoder->sendMsgPush(trim); // expecting no response

    ROS_INFO("TRIM xtalt: %#x", msg.xtalt);
}

void Parser::associationResponseCallback(const atlas_msgs::AssociationResponse& msg)
{
    ROS_INFO("Scheduling %lx\tPeriod: %u\tReliability: %u\tOffset: %u\tRepetitions: %u", msg.eui, msg.period, msg.reliability, msg.offset, msg.repetitions);

    protocol::msgFaSTAssociationResponse_t associationResponse;
    associationResponse.eui = (uint32_t) msg.eui;
    associationResponse.reliability_period = (msg.reliability << 5) + msg.period;
    associationResponse.offset = msg.offset;
    associationResponse.repetitions = msg.repetitions;

    m_decoder->sendMsgPush(associationResponse);    // expecting no response

    ros::Duration(0.001).sleep();
}