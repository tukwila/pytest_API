/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ServerTask.h
 * @brief
 *******************************************************************************
 */

#ifndef SERVER_TASK_H
#define SERVER_TASK_H

#include <memory>
#include <deque>

#include "libMessage/common/communicateCom.h"
#include "libMessage/common/ThreadsafeQueue.h"
#include "ServerMsg.h"
#include "ServerSession.h"

namespace roadDBCore
{


class ServerTask
     : public std::enable_shared_from_this<ServerTask>
{
public:
    ServerTask(serverMsgPtr_t messagePrt,
               const string& ip,
               ServerSessionPtr_t sessionPtr);

    ~ServerTask();

    void responseMsg(serverMsgPtr_t resMsgPtr);

    void setMsgHead(const uint64_t msgId,
                    const uint32_t msgModCode,
                    const MSG_TYPE_E msgType);

    char*    getMsgBody();
    uint32_t getMsgBodyLength();
    MSG_TYPE_E getType();
    uint32_t getModule();
    uint64_t getId();
    string getIp();

private:

    string ip_;
    serverMsgPtr_t         messagePrt_;
    ServerSessionPtr_t  sessionPtr_;

};


}

#endif

