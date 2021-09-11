/**
*******************************************************************************
*                       Continental Confidential
*                  Copyright (c) Continental AG. 2018-2018
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
******************************************************************************
* @file   DataReceiverFacade.h
* @brief  head file of DataReceiverFacade 
*******************************************************************************
*/

#include <jsoncpp.hpp>
#include <string>

const unsigned int MAX_MESSAGER_NUMBER_EDMessageQueue = 1000;
const unsigned int MAX_MESSAGER_SIZE_EDMessageQueue = 1024;
const unsigned int MAX_MESSAGER_NUMBER_RESPONSEQueue = 1;
const unsigned int MAX_MESSAGER_SIZE_RESPONSEQueue = 10240;

enum MQ_RESULT_E
{
    MQ_SUCCESS_E = 0,
    MQ_TIMEOUT_E,   //time out, have no get response
    MQ_FULL_E,  //Queue is full
    MQ_ERROR_E,     //other error
};

/****************************************************************************
 @Function      MQMsgRecv

 @Description   errorDiag mode receive the message form the subcomponent, or the response of the error mode

 *  @param [In]  uTimeOut: the unit is ms, 0 it will return immediately, 
 *                           other value means if it will block to timeout or received messaage.
 *  @param [In]  sQueueName: It's form query response sQueueName.
 *  @param [out] voutMsg: received the payload.
 *
 @Return        MQ_SUCCESS_E, means success

 @Cautions      none.
*//***************************************************************************/
MQ_RESULT_E MQReceive(Json::Value &vOutMsg, unsigned int uTimeOut, std::string sQueueName="");

/****************************************************************************
 @Function      MQQuery

 @Description   subcomponent query errorDiag mode 

 *  @param [In]  uTimeOut: the unit is ms, 0 it will return immediately, but maybe the return error.
 *                           other value means if it will block to timeout or received the query result.
 *  @param [out] vRes: query result payload
 *
 @Return        MQ_SUCCESS_E, means success

 @Cautions      none.
*//***************************************************************************/
MQ_RESULT_E MQQuery(unsigned int uTimeOut, Json::Value & vOutMsg);

/****************************************************************************
 @Function      MQMsgSend

 @Description   The errordiag mode response the subcomponent query.
                Important:the function must be call when received each query, 
                    otherwise it will lead resource leaks.

 *  @param [In]  vMsg: the response detail info, it's the payload.
 *  @param [In]  uTimeOut: the unit is ms, 0 it will return immediately, 
 *                           other value means if it will block to timeout or received messaage.
 *  @param [In]  sQueueName: It's form query response sQueueName.
 *
 @Return        MQ_SUCCESS_E, means success

 @Cautions      none.
*//***************************************************************************/
MQ_RESULT_E MQMsgSend(const Json::Value &vMsg, unsigned int uTimeOut, std::string sQueueName="");