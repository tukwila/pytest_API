/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   communicateCom.h
 * @brief
 *******************************************************************************
 */


#ifndef COMMUNICATE_COM_H
#define COMMUNICATE_COM_H

#include "libMessage/common/libMessage.h"


using namespace std;

namespace roadDBCore
{


enum MSG_TYPE_E: uint32_t
{
    MSG_TYPE_SYNC_E  = 0,
    MSG_TYPE_ASYNC_E = 1,
    MSG_TYPE_NORET_SYNC_E  = 2,
    MSG_TYPE_NORET_ASYNC_E  = 3,
    MSG_TYPE_SERVER_E = 4,
    MSG_TYPE_MAX_E
};


/* IMPORTANT: msgHeader_t must be aligned to 8 bytes
 * If adding a new member, align it to 8 bytes
 * if needed, add dummy place-holders to do so
 * */
struct msgHeader_t
{
    uint64_t        msgId;
    uint32_t        msgLength;
    uint32_t        msgModCode;
    uint32_t        msgResult;
    MSG_TYPE_E      msgType;
    char            msgBody[0];
};

}

#endif


