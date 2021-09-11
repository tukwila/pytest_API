/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   libMessage.h
 * @brief
 *******************************************************************************
 */


#ifndef LIB_MESSAGE_H
#define LIB_MESSAGE_H

#include <functional>
#include <memory>
#include "libMessage/common/IMessage.h"

namespace roadDBCore
{


enum { MAX_DATA_LENGTH = 200 * 1024 * 1024 };

enum {
    PORT_MIN = 1024,
    PORT_MAX = 65535
};

enum {
    MOD_ID_BITS = 16,
    MOD_ID_MASK = 0xFFFF
};

enum {
    DAEMON_MODCODE = 0xEFFE,
    DAEMON_PORT = 19999
};

enum MSG_MODULE_E: uint16_t
{
    MSG_MODULE_COM_E = 0,
    MSG_MODULE_SAM_E = 1,
    MSG_MODULE_STORM_E = 2,
    MSG_MODULE_LOC_E = 3,
    MSG_MODULE_MAX
};


/* callback function
 * check the result in rspMsg->getMsgResult(), 0:OK, other: NOK
 * rspMsg body could be nullptr if it is an ACK from socket */
using msgCbFunc = std::function<void(std::shared_ptr<IMessage> rspMsg)>;


}

#endif


