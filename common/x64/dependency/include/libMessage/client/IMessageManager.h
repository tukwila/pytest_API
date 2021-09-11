/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   IMessageManager.h
 * @brief  interface class of MessageClient.
 *******************************************************************************
 */

#ifndef IMESSAGE_MANAGER_H
#define IMESSAGE_MANAGER_H

#include <cstdio>
#include <cstring>
#include <memory>

#include "libMessage/common/libMessage.h"
#include "libMessage/client/IMessageClient.h"



namespace roadDBCore
{

class IMessageManager
{
public:
    /**
    *******************************************************************************
    * @brief: create IMessageClient by string pair of serverIP:serverPort,
    *         and accept to bind to a localPort
    *
    *  <1> Parameter Description:
    *  @param IN std::string serverIP, string of serverIP
    *            std::string serverPort, string of serverPort
    *            std::string localPort, string of localPort, if omitted, it connects with a random local port
    *
    *  @return IMessageClient*, IMessageClient pointer
    *******************************************************************************
    */
    virtual IMessageClient* createClient(std::string serverIP, std::string serverPort,
                std::string localPort = "") = 0;

    /**
    *******************************************************************************
    * @brief: get IMessageClient pointer by string pair of serverIP:serverPort
    *
    *  <1> Parameter Description:
    *  @param IN std::string serverIP, string of serverIP
    *            std::string serverPort, string of serverPort
    *
    *  @return IMessageClient*, IMessageClient pointer;
    *   return nullptr if it doesn't exist
    *******************************************************************************
    */
    virtual IMessageClient* getClient(std::string serverIP, std::string serverPort) = 0;

    /**
    *******************************************************************************
    * @brief: get the first IMessageClient in manager
    *
    *  <1> Parameter Description:
    *
    *  @return IMessageClient*, IMessageClient pointer
    *******************************************************************************
    */
    virtual IMessageClient* getFirstClient() = 0;

    /**
    *******************************************************************************
    * @brief: start Daemon thread to capture server IP and port
    *
    *  <1> Parameter Description:
    *
    *  @return result
    *******************************************************************************
    */
    virtual uint32_t startDaemon() = 0;

    /**
    *******************************************************************************
    * @brief: stop Daemon thread
    *
    *  <1> Parameter Description:
    *
    *  @ NO return
    *******************************************************************************
    */
    virtual void stopDaemon() = 0;
};

}

#endif
