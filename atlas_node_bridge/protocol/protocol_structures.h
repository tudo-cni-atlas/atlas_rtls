//
//  protocol_structures.h
//  atlas
//
//  Created by Janis on 26.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__protocol_structures__
#define __atlas__protocol_structures__

namespace protocol
{

#pragma pack(1)

typedef struct
{
    uint64_t txId;
    uint64_t rxId;
    uint64_t rxTs;
    uint8_t seqNr;
} msgToa_t;

typedef struct
{
    uint64_t txId;
    uint64_t rxId;
    uint64_t rxTs;
    uint8_t seqNr;
    uint16_t maxNoise;
    uint16_t firstPathAmp1;
    uint16_t stdNoise;
    uint16_t firstPathAmp2;
    uint16_t firstPathAmp3;
    uint16_t maxGrowthCIR;
    uint16_t rxPreamCount;
    uint16_t firstPath;
} msgToaLde_t;

typedef struct
{
    uint64_t txId;
    uint64_t rxId;
    uint64_t rxTs;
    uint8_t seqNr;
    int16_t acc[3];
    int16_t gyr[3];
    int16_t mag[3];
    int16_t temp;
} msgToaImu_t;

typedef struct
{
    uint64_t txId;
    uint64_t rxId;
    uint64_t rxTs;
    uint8_t seqNr;
    int16_t acc[3];
    int16_t gyr[3];
    int16_t mag[3];
    int16_t temp;
    uint16_t maxNoise;
    uint16_t firstPathAmp1;
    uint16_t stdNoise;
    uint16_t firstPathAmp2;
    uint16_t firstPathAmp3;
    uint16_t maxGrowthCIR;
    uint16_t rxPreamCount;
    uint16_t firstPath;
} msgToaImuLde_t;


#pragma pack(1)
typedef struct
{
    uint64_t    eui;
    uint8_t     mode;
    uint8_t     channel;
} msgConfig_t;

typedef struct
{
    uint32_t    slotDuration;
    uint16_t    syncPeriod;
} msgConfigSync_t;

typedef struct
{
    uint32_t    slotDuration;
    uint16_t    syncPeriod;
    uint64_t    masterEui;
    uint16_t    masterOffset;
    uint8_t     randomAccessSlots;
} msgConfigSyncAnchor_t;

typedef struct
{
    uint32_t    slotDuration;
    uint16_t    posPeriod;
} msgConfigRandomTag_t;

typedef struct
{
    uint32_t    slotDuration;
    uint16_t    syncPeriod;
    uint16_t    posPeriod;
    uint16_t    posOffset;
    uint16_t    repetitions;
} msgConfigSyncTag_t;

typedef struct
{
    uint32_t     eui;
    uint8_t      reliability_period;
    uint16_t     offset;
    uint16_t     repetitions;
} msgFaSTAssociationResponse_t;

typedef struct
{
    uint8_t     reporting;
} msgSetReporting_t;

typedef struct
{
    uint8_t     clockTrim;
} msgSetClockTrim_t;

typedef struct
{
    uint64_t    txOffset;
} msgSetTxOffset_t;


}

#endif /* defined(__atlas__protocol_structures__) */