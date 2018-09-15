//
//  protocol.h
//  atlas
//
//  Created by Janis on 26.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__protocol__
#define __atlas__protocol__

#include <stdio.h>
#include <string>
#include <iostream>

#include <stdint.h>
#include "protocol_structures.h"

#include "serial.h"

namespace protocol
{

const uint16_t payloadSize = 0x5FF;

#pragma pack(1)
typedef struct
{
    uint16_t preamble;
    uint16_t messageId;
    uint16_t payloadLength;
    uint8_t payload[payloadSize];
    uint16_t checksum;
} packet_t;

typedef enum
{
    DECODER_STATE_PREAMBLE_0 = 0,
    DECODER_STATE_PREAMBLE_1,
    DECODER_STATE_CLASS_ID,
    DECODER_STATE_MSG_ID,
    DECODER_STATE_LENGTH_0,
    DECODER_STATE_LENGTH_1,
    DECODER_STATE_PAYLOAD,
    DECODER_STATE_CHECKSUM_0,
    DECODER_STATE_CHECKSUM_1
} decoderState_t;

class Protocol
{
public:
    Protocol(std::string port, std::string name);
    ~Protocol();

    void flush(void);

    bool poll(packet_t *pkt);
    uint16_t calculateCheckSum(const packet_t *packet) const;

    template<typename T>
    bool sendMsg(T msg);

    template<typename T>
    bool sendMsgPoll(T *msg);

    template<typename T>
    bool sendMsgPush(T msg);

private:
    Serial m_port;
    std::string m_name;
    packet_t m_packet;
    decoderState_t m_state = DECODER_STATE_PREAMBLE_0;
    uint16_t m_payloadCounter = 0;

    bool sendMessage(packet_t *packet);

    bool checkPacket(const packet_t *packet) const;
    bool getPacket(packet_t *packet, uint32_t timeout);
    bool evaluatePacket(packet_t *packet);
    void populateHeader(packet_t *packet, const uint16_t messageId, const uint16_t payloadLength) const;
};

}


#endif /* defined(__atlas__protocol__) */
