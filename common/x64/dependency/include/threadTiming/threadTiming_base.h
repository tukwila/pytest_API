/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018-2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   threadTiming_base.h
 * @brief
 *******************************************************************************
 */


#ifndef THREAD_TIMING_BASE_H
#define THREAD_TIMING_BASE_H

#include <vector>
#include <mutex>


namespace roadDBCore
{

struct timingTotalInfo_t
{
    char systemName[19];
    uint8_t totalThdNum;
    uint32_t magicInfo;
};

struct timingNodeName_t
{
    uint8_t thdId;        // thread ID
    char thdName[19];     // thread name string, max 18 charater
    uint32_t mainModNum;
};

struct timingMainNodeName_t
{
    uint8_t mainModId;      //0: begin, 0xFF: end
    char mainModName[19];   // main mode name string, max 18 charater
};

struct timingNode_t
{
    uint32_t timeStamp;
    uint8_t thdId;        // thread ID
    uint8_t mainMod;      //0: begin, 0xFF: end
    uint16_t subMod;      //0: begin, 0xFFFF: end
    int32_t frameId;      // frame id

    timingNode_t()
        : timeStamp(0), thdId(0), mainMod(0), subMod(0), frameId(-1)
    {
    }
};

// set TIMING_BUFFER_SIZE as a multiple of timingNode_t size
enum timingBufferSize_e: uint32_t
{
    TIMING_BUFFER_SIZE_1 =           1024*sizeof(timingNode_t),  // 8K
    TIMING_BUFFER_SIZE_2 =        16*1024*sizeof(timingNode_t),  // 128K
    TIMING_BUFFER_SIZE_3 =       256*1024*sizeof(timingNode_t),  // 2M
    TIMING_BUFFER_SIZE_4 =      1024*1024*sizeof(timingNode_t),  // 8M
    TIMING_BUFFER_SIZE_5 =    4*1024*1024*sizeof(timingNode_t),  // 32M
    TIMING_BUFFER_SIZE_6 =   16*1024*1024*sizeof(timingNode_t),  // 128M
    TIMING_BUFFER_SIZE_MAX = 64*1024*1024*sizeof(timingNode_t)   // 512M
};

enum timingState_e : uint16_t
{
    TIMING_BEGIN = 0,
    TIMING_END = 0xFFFF
};

}

#endif


