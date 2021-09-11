/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   MessageObj.h
 * @brief
 *******************************************************************************
 */

#ifndef MESSAGE_OBJ_H
#define MESSAGE_OBJ_H

#include <memory>
#include "libMessage/common/IMessage.h"
#include "libMessage/common/communicateCom.h"


namespace roadDBCore
{

class MessageObj : public IMessage
{
public:
    // create a blank message object with body size = 0
    explicit MessageObj(uint32_t modCode);

    // it will new a buffer of dataLength in msg body, and then copy the data
    MessageObj(char* data, const uint32_t dataLength, const uint32_t modCode, const bool copyData = true);

    // create a blank message object with body size = bodyLength
    MessageObj(const uint32_t bodyLength, const uint32_t modCode);

    virtual ~MessageObj();

    char* getMsgHeader();
    char* getMsgBody();

    uint64_t getMsgId();
    uint32_t getModCode();
    uint16_t getModuleId();
    MSG_TYPE_E getMsgType();
    uint32_t getMsgLength();
    uint32_t getMsgHeaderLength();
    uint32_t getMsgBodyLength();
    uint32_t getMsgResult();

    bool isDataCopied();

    void setMsgHeader(const uint64_t msgId, const MSG_TYPE_E msgType);
    void setMsgBody(char* data, const uint32_t dataLength);
    void setMsgResult(uint32_t result);

private:
    MessageObj(const MessageObj&) = delete;
    MessageObj& operator = (const MessageObj&) = delete;

    // bool copyData_: true to copy the data to msg body, false: don't copy data, just send the data
    bool copyData_ = false;
    uint32_t msgLength_ = 0;
    msgHeader_t* msgHeader_ = nullptr;
    char* msgBody_ = nullptr;

    // /* constant values */
};

}

#endif


