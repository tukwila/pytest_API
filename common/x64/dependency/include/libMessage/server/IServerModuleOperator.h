/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   IServerModuleOperator.h
 * @brief
 *******************************************************************************
 */


#ifndef I_SERVER_MODULE_OPERATOR_H
#define I_SERVER_MODULE_OPERATOR_H

#include "ServerMsg.h"

namespace roadDBCore
{

class IServerModuleOperator
{
public:
    IServerModuleOperator(uint32_t maxParallelTaskNum = 5, uint32_t maxTaskInQueueNum = 10);
    virtual ~IServerModuleOperator();

    uint32_t getMaxParallelTaskNum();
    uint32_t getMaxTaskInQueueNum();

    /**
     *******************************************************************************
     * @brief syncRetFunction - return Synchronously, have reture value
     *
     *  <1> Parameter Description:
     *  @param [In]  - ipStr  client IP adress.
     *  @param [In]  - recMsgLenth  the traffic sign
     *  @recMsgBody [In] - receive message body
     *  @return    the message return to client
     *
     *  <2> Detailed Description:
     *  user shall not care the memory free
     *******************************************************************************
     */
    virtual serverMsgPtr_t syncRetFunction(const string& ipStr,
                                           const uint32_t recMsgLenth,
                                           const char *recMsgBody);

    /**
     *******************************************************************************
     * @brief asyncRetFunction - return Asynchronously, have reture value
     *
     *  <1> Parameter Description:
     *  @param [In]  - ipStr  client IP adress.
     *  @param [In]  - recMsgLenth  the traffic sign
     *  @recMsgBody [In] - receive message body
     *  @return    the message return to client
     *
     *  <2> Detailed Description:
     *  user shall not care the memory free
     *******************************************************************************
     */
    virtual serverMsgPtr_t asyncRetFunction(const string& ipStr,
                                               const uint32_t recMsgLenth,
                                               const char *recMsgBody);

    /**
     *******************************************************************************
     * @brief syncRetFunction - return Synchronously, no reture value
     *
     *  <1> Parameter Description:
     *  @param [In]  - ipStr  client IP adress.
     *  @param [In]  - recMsgLenth  the traffic sign
     *  @recMsgBody [In] - receive message body
     *  @return    void
     *
     *  <2> Detailed Description:
     *  user shall not care the memory free
     *******************************************************************************
     */
    virtual void syncNoRetFunction(const string& ipStr,
                                   const uint32_t recMsgLenth,
                                   const char *recMsgBody);

    /**
      *******************************************************************************
      * @brief asyncRetFunction - return Synchronously, no reture value
      *
      *  <1> Parameter Description:
      *  @param [In]  - ipStr  client IP adress.
      *  @param [In]  - recMsgLenth  the traffic sign
      *  @recMsgBody [In] - receive message body
      *  @return    void
      *
      *  <2> Detailed Description:
      *  user shall not care the memory free
      *******************************************************************************
      */
    virtual void asyncNoRetFunction(const string& ipStr,
                                    const uint32_t recMsgLenth,
                                    const char *recMsgBody);

private:
    uint32_t maxParallelTaskNum_;
    uint32_t maxTaskInQueueNum_;
};

using serverModuleOperatorPtr_t = std::shared_ptr<IServerModuleOperator>;

}

#endif

