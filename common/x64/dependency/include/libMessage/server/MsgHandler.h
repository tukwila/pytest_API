/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   MsgHandler.h
 * @brief
 *******************************************************************************
 */

#ifndef MSG_HANDLER_H
#define MSG_HANDLER_H

#include <memory>
#include "boostThread/threadpool.hpp"
#include "ServerMsg.h"
#include "ServerTask.h"
#include "IServerModuleOperator.h"

namespace roadDBCore
{

using boost::threadpool::pool;

using registerRetFun_t = std::function< serverMsgPtr_t(const string&, //ipStr
                                                          const uint32_t, //recMsgLenth
                                                          char *) >; //recMsgBody

using registerNoRetFun_t = std::function< void(const string&, //ipStr
                                               const uint32_t, //recMsgLenth
                                               char *) >; //recMsgBody

class MsgHandler;
using msgHandlerPtr_t = shared_ptr<MsgHandler>;

class MsgHandler
    : public std::enable_shared_from_this<MsgHandler>

{
public:
    MsgHandler(pool& threadPool,
               const uint32_t modCode,
               const serverModuleOperatorPtr_t serverOperator);

    ~MsgHandler();

    void scheduleMsg(ServerTaskPtr_t msg);

private:
    void dealMassage(ServerTaskPtr_t msg);

    std::atomic<uint32_t> parallelTaskNum_;
    const uint32_t modCode_;
    pool& syncHandleTp_;
    serverModuleOperatorPtr_t serverOperatorPtr_;
    const uint32_t maxParallelTaskNum_;
    const uint32_t maxTaskInQueueNum_;

    ThreadsafeQueue<ServerTaskPtr_t> msgQueue_;
};

}
#endif
