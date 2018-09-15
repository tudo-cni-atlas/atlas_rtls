#include "protocol.h"
#include "protocol_structures.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "ros/ros.h"

using namespace protocol;

const uint16_t msg_id_toa = 0x0101;
const uint16_t msg_id_toa_lde = 0x0102;
const uint16_t msg_id_toa_diag = 0x0103;
const uint16_t msg_id_toa_imu = 0x0104;
const uint16_t msg_id_toa_imu_lde = 0x0105;


const uint16_t msg_id_config = 0x0201;
const uint16_t msg_id_config_sync = 0x0202;
const uint16_t msg_id_config_sync_anchor = 0x0203;
const uint16_t msg_id_config_rtag = 0x0204;
const uint16_t msg_id_config_stag = 0x0205;

const uint16_t msg_id_set_reporting = 0x0301;
const uint16_t msg_id_set_clock_trim = 0x0302;
const uint16_t msg_id_set_tx_offset = 0x0303;

const uint16_t msg_id_association_rsp = 0x401;



template<typename T> inline uint16_t getMsgId(T t)
{
    return 0;
}

template<> inline uint16_t getMsgId<msgToa_t>(msgToa_t t)
{
    return msg_id_toa;
}
template<> inline uint16_t getMsgId<msgToaLde_t>(msgToaLde_t t)
{
    return msg_id_toa_lde;
}
template<> inline uint16_t getMsgId<msgToaImu_t>(msgToaImu_t t)
{
    return msg_id_toa_imu;
}
template<> inline uint16_t getMsgId<msgToaImuLde_t>(msgToaImuLde_t t)
{
    return msg_id_toa_imu_lde;
}

template<> inline uint16_t getMsgId<msgConfig_t>(msgConfig_t t)
{
    return msg_id_config;
}
template<> inline uint16_t getMsgId<msgConfigSync_t>(msgConfigSync_t t)
{
    return msg_id_config_sync;
}
template<> inline uint16_t getMsgId<msgConfigSyncAnchor_t>(msgConfigSyncAnchor_t t)
{
    return msg_id_config_sync_anchor;
}
template<> inline uint16_t getMsgId<msgConfigRandomTag_t>(msgConfigRandomTag_t t)
{
    return msg_id_config_rtag;
}
template<> inline uint16_t getMsgId<msgConfigSyncTag_t>(msgConfigSyncTag_t t)
{
    return msg_id_config_stag;
}
template<> inline uint16_t getMsgId<msgFaSTAssociationResponse_t>(msgFaSTAssociationResponse_t t)
{
    return msg_id_association_rsp;
}

template<> inline uint16_t getMsgId<msgSetReporting_t>(msgSetReporting_t t)
{
    return msg_id_set_reporting;
}
template<> inline uint16_t getMsgId<msgSetClockTrim_t>(msgSetClockTrim_t t)
{
    return msg_id_set_clock_trim;
}
template<> inline uint16_t getMsgId<msgSetTxOffset_t>(msgSetTxOffset_t t)
{
    return msg_id_set_tx_offset;
}




template<typename T>
bool Protocol::sendMsg(T msg)
{
    uint16_t msg_id = getMsgId(msg);

    packet_t packet;
    populateHeader(&packet, msg_id, sizeof(T));
    memcpy(packet.payload, &msg, sizeof(T));
    packet.checksum = calculateCheckSum(&packet);
    sendMessage(&packet);

    uint32_t timeout = 1000;
    if(!getPacket(&packet, timeout))
    {
        ROS_ERROR("!! %s Timeout occurred after %d ms", m_name.c_str(), timeout);
        return false;
    }

    if (calculateCheckSum(&packet) != packet.checksum)
    {
        ROS_ERROR("!! %s Checksum Error", m_name.c_str());
        return false;
    }

    return true;
}

template<typename T>
bool Protocol::sendMsgPoll(T *msg)
{
    uint16_t msg_id = getMsgId(*msg);

    packet_t packet;
    populateHeader(&packet, msg_id, 0);
    packet.checksum = calculateCheckSum(&packet);
    sendMessage(&packet);

    uint32_t timeout = 1000;
    if(!getPacket(&packet, timeout))
    {
        ROS_ERROR("!! %s Timeout occurred after %d ms", m_name.c_str(), timeout);
        return false;
    }

    if (calculateCheckSum(&packet) != packet.checksum)
    {
        ROS_ERROR("!! %s Checksum Error", m_name.c_str());
        return false;
    }

    memcpy(msg, packet.payload, sizeof(T));
    return true;
}

template<typename T>
bool Protocol::sendMsgPush(T msg)
{
    uint16_t msg_id = getMsgId(msg);

    packet_t packet;
    populateHeader(&packet, msg_id, sizeof(T));
    memcpy(packet.payload, &msg, sizeof(T));
    packet.checksum = calculateCheckSum(&packet);
    sendMessage(&packet);
    return true;
}

template bool Protocol::sendMsg<msgToa_t>(msgToa_t msg);
template bool Protocol::sendMsg<msgToaLde_t>(msgToaLde_t msg);
//template bool Protocol::sendMsg<msgToaDiag_t>(msgToaDiag_t msg);
template bool Protocol::sendMsg<msgToaImu_t>(msgToaImu_t msg);
template bool Protocol::sendMsg<msgToaImuLde_t>(msgToaImuLde_t msg);

template bool Protocol::sendMsg<msgConfig_t>(msgConfig_t msg);
template bool Protocol::sendMsg<msgConfigSync_t>(msgConfigSync_t msg);
template bool Protocol::sendMsg<msgConfigSyncAnchor_t>(msgConfigSyncAnchor_t msg);
template bool Protocol::sendMsg<msgConfigRandomTag_t>(msgConfigRandomTag_t msg);
template bool Protocol::sendMsg<msgConfigSyncTag_t>(msgConfigSyncTag_t msg);
template bool Protocol::sendMsg<msgSetReporting_t>(msgSetReporting_t msg);
template bool Protocol::sendMsg<msgSetClockTrim_t>(msgSetClockTrim_t msg);
template bool Protocol::sendMsg<msgSetTxOffset_t>(msgSetTxOffset_t msg);

template bool Protocol::sendMsgPoll<msgToa_t>(msgToa_t *msg);
template bool Protocol::sendMsgPoll<msgToaLde_t>(msgToaLde_t *msg);
//template bool Protocol::sendMsgPoll<msgToaDiag_t>(msgToaDiag_t *msg);
template bool Protocol::sendMsgPoll<msgToaImu_t>(msgToaImu_t *msg);
template bool Protocol::sendMsgPoll<msgToaImuLde_t>(msgToaImuLde_t *msg);
template bool Protocol::sendMsgPoll<msgConfig_t>(msgConfig_t *msg);
template bool Protocol::sendMsgPoll<msgConfigSync_t>(msgConfigSync_t *msg);
template bool Protocol::sendMsgPoll<msgConfigSyncAnchor_t>(msgConfigSyncAnchor_t *msg);
template bool Protocol::sendMsgPoll<msgConfigRandomTag_t>(msgConfigRandomTag_t *msg);
template bool Protocol::sendMsgPoll<msgConfigSyncTag_t>(msgConfigSyncTag_t *msg);
template bool Protocol::sendMsgPoll<msgSetReporting_t>(msgSetReporting_t *msg);
template bool Protocol::sendMsgPoll<msgSetClockTrim_t>(msgSetClockTrim_t *msg);
template bool Protocol::sendMsgPoll<msgSetTxOffset_t>(msgSetTxOffset_t *msg);

template bool Protocol::sendMsgPush<msgToa_t>(msgToa_t msg);
template bool Protocol::sendMsgPush<msgToaLde_t>(msgToaLde_t msg);
//template bool Protocol::sendMsgPush<msgToaDiag_t>(msgToaDiag_t msg);
template bool Protocol::sendMsgPush<msgToaImu_t>(msgToaImu_t msg);
template bool Protocol::sendMsgPush<msgToaImuLde_t>(msgToaImuLde_t msg);
template bool Protocol::sendMsgPush<msgConfig_t>(msgConfig_t msg);
template bool Protocol::sendMsgPush<msgConfigSync_t>(msgConfigSync_t msg);
template bool Protocol::sendMsgPush<msgConfigSyncAnchor_t>(msgConfigSyncAnchor_t msg);
template bool Protocol::sendMsgPush<msgConfigRandomTag_t>(msgConfigRandomTag_t msg);
template bool Protocol::sendMsgPush<msgConfigSyncTag_t>(msgConfigSyncTag_t msg);
template bool Protocol::sendMsgPush<msgSetReporting_t>(msgSetReporting_t msg);
template bool Protocol::sendMsgPush<msgSetClockTrim_t>(msgSetClockTrim_t msg);
template bool Protocol::sendMsgPush<msgSetTxOffset_t>(msgSetTxOffset_t msg);
