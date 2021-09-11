/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ClientManager.h
 * @brief  class ClientManager
 *******************************************************************************
 */

#ifndef CLIENT_MANAGER_H
#define CLIENT_MANAGER_H

#include <boost/asio.hpp>
#include "libMessage/client/message_base.h"
#include "libMessage/client/IMessageManager.h"
#include "libMessage/client/ClientDaemonServer.h"
#include "libMessage/client/MessageClient.h"


namespace roadDBCore
{

class ClientDaemonServer;

class ClientManager : public IMessageManager
{
public:
    static ClientManager* createDeleteInstance(bool bCreate);
    virtual ~ClientManager();

    uint32_t startDaemon();
    void stopDaemon();

    IMessageClient* createClient(std::string serverIP, std::string serverPort, std::string localPort = "");
    IMessageClient* getClient(std::string serverIP, std::string serverPort);
    IMessageClient* getFirstClient();

    void notify(std::pair<std::string, std::string> sp);

protected:
    ClientManager();

private:
    void managerRun();

private:
    int32_t daemonStatus_;
    ClientDaemonServer daemonServer_;

    // <"serverIP", "port"> : MessageClient
    std::map<std::pair<std::string, std::string>, MessageClient*> mSP2MsgClient_;

    // vector of <"serverIP", "port"> pair, get from daemonServer_
    // will be clear after creating MessageClient accordingly
    std::vector<std::pair<std::string, std::string>> vSP_;

    std::thread managerThread_;
    volatile bool stopManagerThd_;
    // protect manager
    std::mutex managerMtx_;

    // synchronize with daemonServer_
    Semaphore managerSem_;
};

}

#endif
