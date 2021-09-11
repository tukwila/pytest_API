/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017-2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   MessageClientAgent.h
 * @brief  class MessageClientAgent.
 *******************************************************************************
 */

#ifndef MESSAGE_CLIENT_AGENT_H
#define MESSAGE_CLIENT_AGENT_H

#include "libMessage/client/IMessageManager.h"

namespace roadDBCore
{

class MessageClientAgent
{
public:
    static IMessageManager* getClientManager();

    static void releaseClientManager();
};

}

#endif // MESSAGE_CLIENT_AGENT_H
