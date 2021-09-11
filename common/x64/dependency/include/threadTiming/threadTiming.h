/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018-2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   threadTiming.h
 * @brief
 *******************************************************************************
 */


#ifndef THREAD_TIMING_H
#define THREAD_TIMING_H

#include "threadTiming/threadTiming_base.h"


namespace roadDBCore
{

void timingSetBufSize(timingBufferSize_e bufferSize = TIMING_BUFFER_SIZE_4);

// mainMod and subMod both 0: begin, both 0xFF: end
void timingNow(uint8_t thdId, uint8_t mainMod, uint16_t subMod, int32_t frameId = -1);

inline void timingStart(uint8_t thdId, int32_t frameId = -1) {timingNow(thdId, 0, 0, frameId);}

inline void timingEnd(uint8_t thdId, int32_t frameId = -1) {timingNow(thdId, 0xFF, 0xFFFF, frameId);}

void timingDumpHead(char *pBuffer, size_t bufferLen, const std::string &fileName = "./threadTiming.bin");

void timingDump(const std::string &fileName = "./threadTiming.bin");

}

#endif


