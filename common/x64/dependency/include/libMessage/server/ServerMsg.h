/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ServerMsg.h
 * @brief
 *******************************************************************************
 */

#ifndef SERVER_MSG_H
#define SERVER_MSG_H

#include<memory>
#include "libMessage/common/communicateCom.h"

namespace roadDBCore
{

class ServerMsg
{

public:
    ServerMsg(const char* const data, const uint32_t length);
    ~ServerMsg();

    void setHead(const uint64_t msgId,
                 const uint32_t msgModCode,
                 const MSG_TYPE_E msgType);

    void setResult(uint32_t result);

    char* getMsgBody();
    uint32_t getMsgBodyLength();

    char* getMsgData();
    uint32_t getMsgDataLength();

    uint64_t getMsgId();
    uint32_t getMsgModCode();
    MSG_TYPE_E getMsgType();

private:
    ServerMsg(const ServerMsg &serverMsg);
    ServerMsg& operator =(const ServerMsg &serverMsg);

private:
    uint32_t msgLength_;
    msgHeader_t* msgData_;


};


using serverMsgPtr_t = std::shared_ptr<ServerMsg>;

}
#endif


