/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ServerSession.h
 * @brief
 *******************************************************************************
 */

#ifndef SERVER_SESSION_H
#define SERVER_SESSION_H

#include <set>
#include <boost/asio.hpp>

#include "libMessage/common/ThreadsafeQueue.h"
#include "ServerTask.h"

namespace roadDBCore
{

using boost::asio::ip::tcp;

class ServerTask;
using ServerTaskPtr_t = std::shared_ptr<ServerTask>;

class ServerSession;
using ServerSessionPtr_t = std::shared_ptr<ServerSession>;
using serverSessionSet = std::set<ServerSessionPtr_t>;

struct sessionMap_t
{
    std::mutex sessionMtx;
    std::map<string, serverSessionSet> ssMap;
};

class ServerSession
     : public std::enable_shared_from_this<ServerSession>
{
public:
    ServerSession(tcp::socket socket, sessionMap_t& sessionMap);
    ~ServerSession();

    void start();

    void responseMsg(serverMsgPtr_t resMsgPtr);

    void close();

    string getIp();

    uint16_t getPort();


private:
    void readHeader();
    void readBody();
    void sendMsgToClient();
    tcp::socket socket_;
    sessionMap_t& sessionMap_;
//    std::map<string, serverSessionSet>& sessionMap_;
    msgHeader_t headBuff_;
    ThreadsafeQueue<serverMsgPtr_t> msgQueue_;

    bool isStart_;
    // protect isStart_
    std::mutex startMtx_;

    std::atomic<uint32_t> isSending_;
    string ip_;
    uint16_t port_;

};

}
#endif



