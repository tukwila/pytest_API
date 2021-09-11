/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ClientDaemonServer.h
 * @brief  class ClientDaemonServer
 *******************************************************************************
 */

#ifndef CLIENT_DAEMON_SERVER_H
#define CLIENT_DAEMON_SERVER_H

#include <boost/asio.hpp>
#include "LogWrapper/LogWrapper.h"
#include "libMessage/server/CommunicateServer.h"
#include "libMessage/server/IServerModuleOperator.h"
#include "libMessage/client/message_base.h"


namespace roadDBCore
{

class ClientManager;

class ClientDaemonServer;


class DaemonServerOperator : public IServerModuleOperator
{
public:
    DaemonServerOperator(ClientDaemonServer *daemonSvr);

    void syncNoRetFunction(const string& ipStr,
                           const uint32_t recMsgLenth,
                           const char *recMsgBody);
private:
    ClientDaemonServer *daemonSvr_;
};


class ClientDaemonServer
{
public:
    ClientDaemonServer(ClientManager *manager);
    virtual ~ClientDaemonServer();

    uint32_t start();
    void stop();

protected:
    friend class DaemonServerOperator;

private:
    int32_t status_;

    ClientManager *clientManager_;
    CommunicateServer server_;
    std::mutex daemon_mtx_;
    std::thread daemonThread_;

    std::shared_ptr<DaemonServerOperator> serverOptor_;

    uint32_t daemonHit_;
    std::pair<std::string, std::string> sp_;

//    const uint32_t DaemonPort_ = 19999;
};

}

#endif
