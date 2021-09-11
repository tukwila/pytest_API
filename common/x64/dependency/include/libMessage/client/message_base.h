/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   message_base.h
 * @brief  base header file for class MessageClient
 *******************************************************************************
 */

#ifndef MESSAGE_BASE_H
#define MESSAGE_BASE_H

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <thread>

#include "libMessage/common/communicateCom.h"
#include "libMessage/common/libMessage.h"
#include "libMessage/client/MessageObj.h"

namespace roadDBCore
{

//enum { MAX_DATA_LENGTH = 10240 };


typedef std::deque<std::shared_ptr<MessageObj>> msgQueue;


enum { SINGLE_LOOP_MAX_TIME_OUT = 20 };

class Semaphore
{
public:
    Semaphore ()
        : ready(false) {}

    inline void notify()
    {
        std::unique_lock<std::mutex> lock(mtx);
        ready = true;
        cv.notify_one();
    }

    inline void wait()
    {
        std::unique_lock<std::mutex> lock(mtx);

        while(!ready)
        {
            cv.wait(lock);
        }
        ready = false;
    }

    inline void wait(int32_t msNum)
    {
        if(msNum <= 0)
        {
            return wait();
        }

        std::chrono::milliseconds msNm(msNum);
        std::unique_lock<std::mutex> lock(mtx);

        cv.wait_for(lock, msNm);
        if(ready)
        {
            ready = false;
        }
    }

private:
    std::mutex mtx;
    std::condition_variable cv;
    bool ready;
};


enum CONNT_STA_E : uint32_t
{
    CONNT_STA_NOT_CONNT = 0,
    CONNT_STA_CONNTING = 1,
    CONNT_STA_CONNTED = 2
};


enum CONNT_ACK_E: uint32_t
{
    CONNT_ACK_NO = 0,
    CONNT_ACK_SOCKET = 1,
    CONNT_ACK_SERVER = 2,
    CONNT_ACK_MAX
};

struct msgCBInfo_t
{
    Semaphore sem;
    CONNT_ACK_E gotAck;
    msgCbFunc cb;
    int32_t result;
    MSG_TYPE_E msgType;
};

struct modContext_t
{
    msgQueue modMsgQue;
    std::shared_ptr<std::thread> modThread;
    Semaphore sem;
    bool started;
    bool stopFlag;
};

}

#endif // MESSAGE_BASE_H
