/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   IMessageClient.h
 * @brief  interface class of MessageClient.
 *******************************************************************************
 */

#ifndef IMESSAGE_CLIENT_H
#define IMESSAGE_CLIENT_H

#include <cstdio>
#include <cstring>
#include <memory>

#include "libMessage/common/libMessage.h"


namespace roadDBCore
{

class IMessageClient
{
public:

    /**
    *******************************************************************************
    * @brief: send msg synchronously to server, return until get the response msg from server
    *
    *  <1> Parameter Description:
    *  @param IN char* data, data to send
    *         IN const int32_t dataLength, data length
    *         IN const uint32_t modCode, module code: module Id (16bit << 16) | module msg type (16bit)
    *         OUT std::shared_ptr<IMessage> &rspMsg, response msg from server
    *
    *  @return the result of process, 0:OK, other: NOK
    *******************************************************************************
    */
    /* it WON'T copy the data, the lifecycle of which is maintained by user*/
    virtual uint32_t syncSend(char* data, const int32_t dataLength, const uint32_t modCode,
                std::shared_ptr<IMessage> &rspMsg) = 0;

    /* it WILL copy the data and then send*/
    virtual uint32_t syncSend_Copy(char* data, const int32_t dataLength, const uint32_t modCode,
                std::shared_ptr<IMessage> &rspMsg) = 0;

    /**
    *******************************************************************************
    * @brief: send msg synchronously to server, return until get the ACK of socket;
    *         server won't respond
    *
    *  <1> Parameter Description:
    *  @param IN char* data, data to send
    *         IN const int32_t dataLength, data length
    *         IN const uint32_t modCode, module code: module Id (16bit << 16) | module msg type (16bit)
    *
    *  @return the result of process, 0:OK, other: NOK
    *******************************************************************************
    */
    /* it WON'T copy the data, the lifecycle of which is maintained by user*/
    virtual uint32_t syncSend_NoRsp(char* data, const int32_t dataLength,
                const uint32_t modCode) = 0;

    /* it WILL copy the data and then send*/
    virtual uint32_t syncSend_Copy_NoRsp(char* data, const int32_t dataLength,
                const uint32_t modCode) = 0;

    /**
    *******************************************************************************
    * @brief: send msg asynchronously to server, handle the response msg from server in callback function
    *
    *  <1> Parameter Description:
    *  @param IN char* data, data to send
    *         IN const int32_t dataLength, data length
    *         IN const uint32_t modCode, module code: module Id (16bit << 16) | module msg type (16bit)
    *         msgCbFunc cb, handle the response msg from server in callback function
    *         OUT uint64_t &msgId, set back the msgId
    *
    *  @ return the result of process, 0:OK, other: NOK
    *******************************************************************************
    */
    /* it WON'T copy the data, the lifecycle of which is maintained by user*/
    virtual uint32_t asyncSend(char* data, const int32_t dataLength, const uint32_t modCode,
            msgCbFunc cb, uint64_t &msgId) = 0;

    /* it WILL copy the data and then send*/
    virtual uint32_t asyncSend_Copy(char* data, const int32_t dataLength, const uint32_t modCode,
            msgCbFunc cb, uint64_t &msgId) = 0;

    /**
    *******************************************************************************
    * @brief: send msg asynchronously to server, handle the ACK of socket in callback function;
    *         server won't respond
    *
    *  <1> Parameter Description:
    *  @param IN char* data, data to send
    *         IN const int32_t dataLength, data length
    *         IN const uint32_t modCode, module code: module Id (16bit << 16) | module msg type (16bit)
    *         msgCbFunc cb, handle the ACK of socket in callback function
    *         OUT uint64_t &msgId, set back the msgId
    *
    *  @ return the result of process, 0:OK, other: NOK
    *******************************************************************************
    */
    /* it WON'T copy the data, the lifecycle of which is maintained by user*/
    virtual uint32_t asyncSend_NoRsp(char* data, const int32_t dataLength, const uint32_t modCode,
            msgCbFunc cb, uint64_t &msgId) = 0;

    /* it WILL copy the data and then send*/
    virtual uint32_t asyncSend_Copy_NoRsp(char* data, const int32_t dataLength, const uint32_t modCode,
            msgCbFunc cb, uint64_t &msgId) = 0;

    /**
    *******************************************************************************
    * @brief: Register the callback function to handle server sponsored msg
    *         It can only register for once, it won't register the callback for the second registry.
    *
    *  <1> Parameter Description:
    *  @param IN const uint32_t modCode, module code = (MSG_MODULE_E << 16) + sub code (user defined)
    *         msgCbFunc cb, callback function to handle server sponsored msg
    *
    *  @ return the result of process, 0:OK, other: NOK
    *******************************************************************************
    */
    virtual uint32_t registerSvrMsgCallback(const uint32_t modCode, msgCbFunc cb) = 0;

    /**
    *******************************************************************************
    * @brief: check whether the client is connected to server or not
    *
    *  <1> Parameter Description:
    *
    *  @ return true if connected
    *******************************************************************************
    */
    virtual bool isConnected() = 0;

    /**
    *******************************************************************************
    * @brief: stop the client socket service
    *
    *  <1> Parameter Description:
    *
    *  @ NO return
    *******************************************************************************
    */
    virtual void stop() = 0;
};

}

#endif // IMESSAGE_CLIENT_H
