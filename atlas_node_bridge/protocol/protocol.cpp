#include "protocol.h"
#include "protocol_structures.h"

#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "serial.h"

#include <chrono>

#include "ros/ros.h"

using namespace protocol;


const uint16_t preamble = 0x62B5;


Protocol::Protocol(std::string port, std::string name)
    : m_port(port, 115200)
{
    m_name = name;
}

Protocol::~Protocol()
{

}

void Protocol::flush(void)
{
    m_port.flush();
}

bool Protocol::poll(packet_t *pkt)
{
    bool isWholePacket = false;
    uint8_t c;
    while(true)
    {
        int32_t ret = m_port.receive(&c);
        //std::cout << std::dec << ret << std::endl;
        if(ret == 0)
        {
            //usleep(1);
            break;
        }
        if(ret < 0)
        {
            //usleep(1);
            break;
        }

        //std::cout << std::hex << "x" << (int)c << " ";
        //std::cout << std::flush;
        switch(m_state)
        {
        case DECODER_STATE_PREAMBLE_0:
            //std::cout << "DECODER_STATE_SYNC_0" << std::endl;
            m_state = (c == (preamble & 0xFF)) ? DECODER_STATE_PREAMBLE_1 : DECODER_STATE_PREAMBLE_0;
            break;

        case DECODER_STATE_PREAMBLE_1:
            //std::cout << "DECODER_STATE_SYNC_1" << std::endl;
            m_state = (c == (preamble >> 8)) ? DECODER_STATE_CLASS_ID : DECODER_STATE_PREAMBLE_0;
            break;

        case DECODER_STATE_CLASS_ID:
            //std::cout << "DECODER_STATE_CLASS_ID" << std::endl;
            m_packet.messageId = c;
            m_state = DECODER_STATE_MSG_ID;
            break;

        case DECODER_STATE_MSG_ID:
            //std::cout << "DECODER_STATE_MSG_ID" << std::endl;
            m_packet.messageId += (c << 8);
            m_state = DECODER_STATE_LENGTH_0;
            break;

        case DECODER_STATE_LENGTH_0:
            //std::cout << "DECODER_STATE_LENGTH_0" << std::endl;
            m_packet.payloadLength = c;
            m_state = DECODER_STATE_LENGTH_1;
            break;

        case DECODER_STATE_LENGTH_1:
            //std::cout << "DECODER_STATE_LENGTH_1" << std::endl;
            m_packet.payloadLength += (c << 8);

            if(m_packet.payloadLength > payloadSize)
            {
                return false;
            }

            m_payloadCounter = m_packet.payloadLength;
            if(m_packet.payloadLength > 0)
            {
                m_state = DECODER_STATE_PAYLOAD;
            }
            else
            {
                m_state = DECODER_STATE_CHECKSUM_0;
            }
            break;

        case DECODER_STATE_PAYLOAD:
            //std::cout << "DECODER_STATE_PAYLOAD" << std::endl;
            m_packet.payload[m_packet.payloadLength - m_payloadCounter--] = c;
            if(m_payloadCounter == 0)
            {
                m_state = DECODER_STATE_CHECKSUM_0;
            }
            break;

        case DECODER_STATE_CHECKSUM_0:
            //std::cout << "DECODER_STATE_CHECKSUM_0" << std::endl;
            m_packet.checksum = c;
            m_state = DECODER_STATE_CHECKSUM_1;
            break;

        case DECODER_STATE_CHECKSUM_1:
            //std::cout << "DECODER_STATE_CHECKSUM_1" << std::endl;
            m_packet.checksum += (c << 8);
            m_state = DECODER_STATE_PREAMBLE_0;
            isWholePacket = true;
            *pkt = m_packet;
            //std::cout << "Took " << std::dec << std::chrono::duration_cast<std::chrono::milliseconds>(d).count() << "ms" << std::endl;
            return isWholePacket;
            break;
        }

        //usleep(10);
    }
    //usleep(100);

    return isWholePacket;
}

void Protocol::populateHeader(packet_t *packet,
                              const uint16_t messageId,
                              const uint16_t payloadLength) const
{
    packet->preamble = preamble;
    packet->messageId = messageId;
    packet->payloadLength = payloadLength;
}

bool Protocol::getPacket(packet_t *packet, uint32_t timeout)
{
    decoderState_t state = DECODER_STATE_PREAMBLE_0;
    uint8_t payloadCounter = 0;
    bool isWholePacket = false;

    auto ts = std::chrono::high_resolution_clock::now();

    while (!isWholePacket)
    {
        auto t = std::chrono::high_resolution_clock::now();
        auto d = t - ts;

        if (d > std::chrono::milliseconds(timeout))
        {

            return false;
        }

        uint8_t c;
        while(true)
        {
            int ret = m_port.receive(&c);
            if(ret == 0)
            {
                usleep(1);
                break;
            }
            if(ret < 0)
            {
                usleep(1);
                break;
            }

            //std::cout << std::hex << "x" << (int)c << " ";
            //std::cout << std::flush;
            switch(state)
            {
            case DECODER_STATE_PREAMBLE_0:
                //std::cout << "DECODER_STATE_SYNC_0" << std::endl;
                state = (c == (preamble & 0xFF)) ? DECODER_STATE_PREAMBLE_1 : DECODER_STATE_PREAMBLE_0;
                break;

            case DECODER_STATE_PREAMBLE_1:
                //std::cout << "DECODER_STATE_SYNC_1" << std::endl;
                state = (c == (preamble >> 8)) ? DECODER_STATE_CLASS_ID : DECODER_STATE_PREAMBLE_0;
                break;

            case DECODER_STATE_CLASS_ID:
                //std::cout << "DECODER_STATE_CLASS_ID" << std::endl;
                packet->messageId = c;
                state = DECODER_STATE_MSG_ID;
                break;

            case DECODER_STATE_MSG_ID:
                //std::cout << "DECODER_STATE_MSG_ID" << std::endl;
                packet->messageId += (c << 8);
                state = DECODER_STATE_LENGTH_0;
                break;

            case DECODER_STATE_LENGTH_0:
                //std::cout << "DECODER_STATE_LENGTH_0" << std::endl;
                packet->payloadLength = c;
                state = DECODER_STATE_LENGTH_1;
                break;

            case DECODER_STATE_LENGTH_1:
                //std::cout << "DECODER_STATE_LENGTH_1" << std::endl;
                packet->payloadLength += (c << 8);

                if(packet->payloadLength > payloadSize)
                {
                    return false;
                }

                payloadCounter = packet->payloadLength;
                if(packet->payloadLength > 0)
                {
                    state = DECODER_STATE_PAYLOAD;
                }
                else
                {
                    state = DECODER_STATE_CHECKSUM_0;
                }
                break;

            case DECODER_STATE_PAYLOAD:
                //std::cout << "DECODER_STATE_PAYLOAD" << std::endl;
                packet->payload[packet->payloadLength - payloadCounter--] = c;
                if(payloadCounter == 0)
                {
                    state = DECODER_STATE_CHECKSUM_0;
                }
                break;

            case DECODER_STATE_CHECKSUM_0:
                //std::cout << "DECODER_STATE_CHECKSUM_0" << std::endl;
                packet->checksum = c;
                state = DECODER_STATE_CHECKSUM_1;
                break;

            case DECODER_STATE_CHECKSUM_1:
                //std::cout << "DECODER_STATE_CHECKSUM_1" << std::endl;
                packet->checksum += (c << 8);
                state = DECODER_STATE_PREAMBLE_0;
                isWholePacket = true;
                //std::cout << std::endl << "Received Packet, Took " << std::dec << std::chrono::duration_cast<std::chrono::milliseconds>(d).count() << "ms" << std::endl;
                break;
            }

            //usleep(10);
        }
        //usleep(10);
    }
    return isWholePacket;
}

bool Protocol::sendMessage(packet_t *packet)
{
    uint8_t msg[payloadSize];

    memcpy(msg, packet, 6);
    memcpy(msg + 6, packet->payload, packet->payloadLength);
    memcpy(msg + 6 + packet->payloadLength, &packet->checksum, 2);

    //std::cout << "RAW: ";
    for(uint16_t i = 0; i < 6 + packet->payloadLength + 2; ++i)
    {
        int n = msg[i];
        //std::cout << "x" << std::hex << n << " ";
    }
    //std::cout << std::endl;

    m_port.send(msg, 6 + packet->payloadLength + 2);
    return true;
}

bool Protocol::checkPacket(const packet_t *packet) const
{
    uint16_t cs = calculateCheckSum(packet);
    return (cs == packet->checksum);
}

uint16_t Protocol::calculateCheckSum(const packet_t *packet) const
{
    uint8_t a = 0;
    uint8_t b = 0;

    a = a + (packet->messageId & 0xFF);
    b = b + a;

    a = a + (packet->messageId >> 8);
    b = b + a;

    a = a + (packet->payloadLength & 0xFF);
    b = b + a;

    a = a + (packet->payloadLength >> 8);
    b = b + a;

    for(uint16_t i = 0; i < packet->payloadLength; ++i)
    {
        a = a + packet->payload[i];
        b = b + a;
    }

    return (b << 8) + a;
}
