/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018-2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   CThreadTiming.h
 * @brief
 *******************************************************************************
 */


#ifndef THREAD_INSPECT_H
#define THREAD_INSPECT_H

#include <mutex>
#include <map>
#include <memory>
#include <atomic>
#include "threadTiming/threadTiming_base.h"


namespace roadDBCore
{

class CThreadTiming
{
public:
    static CThreadTiming* instance(timingBufferSize_e bufferSize = TIMING_BUFFER_SIZE_4);

    virtual ~CThreadTiming();
    bool init();

    // record the timing info
    // mainMod and subMod both 0: begin, both 0xFF: end
    void timingNow(uint8_t thdId, uint8_t mainMod, uint16_t subMod, int32_t frameId = -1);
    // dump timing info and thread name
    void dumpHeadInfo(const std::string &fileName, char *pBuffer, size_t bufferLen);
    void dump(const std::string &fileName = "./threadTiming.bin");

    uint32_t getBufSize();

protected:
    CThreadTiming(uint32_t timingNodeNum);

private:
    void dumpTimingInfo(const std::string &fileName, char *pBuffer, size_t bufferLen);
    CThreadTiming(const CThreadTiming &threadTiming);
    CThreadTiming& operator =(const CThreadTiming &threadTiming);

private:
    bool bInit;
    // buffer to store timingNode_t
    timingNode_t *pTimingBuffer;
    // lastId_ is the index of last valid data of pTimingBuffer
    std::atomic<uint32_t> lastId_;
    // protect isDumping_
    std::mutex thdMtx_;

    // keep the timing node number, equal to the vector size of pTimingBuffer
    const uint32_t TimingNodeNum_;
    // save TimingNodeNum_ - 1
    const uint32_t TimingNodeNum_1_;
};

}

#endif


