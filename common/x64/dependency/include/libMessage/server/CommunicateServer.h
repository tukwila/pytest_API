/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   CommunicateServer.h
 * @brief
 *******************************************************************************
 */

#ifndef COMMUNICATE_SERVER_H
#define COMMUNICATE_SERVER_H

#include <set>
#include <map>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include "boostThread/threadpool.hpp"
#include "libMessage/common/communicateCom.h"
#include "MsgHandler.h"
#include "ServerSession.h"
#include "ServerTask.h"

using boost::threadpool::pool;
using boost::asio::ip::tcp;

namespace roadDBCore
{


class CommunicateServer
     :  public std::enable_shared_from_this<CommunicateServer>
{

public:
    CommunicateServer(int32_t port = 9999);

    /**
     *******************************************************************************
     * @brief registMsgOperator - return is regist success
     *
     *  <1> Parameter Description:
     *  @param [In]  - moduleCode   regist moduleCode.
     *  @param [In]  - serverOperator   regist operator.
     *  @return    is regist success
     *
     *  <2> Detailed Description:
     *  one module only can be registed one time.
     *******************************************************************************
     */
    static uint32_t registMsgOperator(const uint32_t moduleCode,
                                      const serverModuleOperatorPtr_t serverOperator);

    /**
     *******************************************************************************
     * @brief sendMsgToClient - return is regist success
     *
     *  <1> Parameter Description:
     *  @param [In]  - moduleCode   message moduleCode.
     *  @param [In]  - sendMsg   send message.
     *  @param [In]  - ip   send to ip.
     *  @return    is regist success
     *
     *  <2> Detailed Description:
     *  one module only can be registed one time.
     *******************************************************************************
     */
    uint32_t sendMsgToClient(const uint32_t moduleCode,
                             serverMsgPtr_t sendMsg,
                             const string& ip);

    /**
     *******************************************************************************
     * @brief sendMsgToClient - return is regist success
     *
     *  <1> Parameter Description:
     *  @param [In]  - moduleCode   message moduleCode.
     *  @param [In]  - sendMsg   send message.
     *  @param [In]  - ip   send to ip.
     *  @param [In]  - port send to port.
     *  @return    is regist success
     *
     *  <2> Detailed Description:
     *  one module only can be registed one time.
     *******************************************************************************
     */
    uint32_t sendMsgToClient(const uint32_t moduleCode,
                             serverMsgPtr_t sendMsg,
                             const string& ip,
                             const uint16_t port);

    /**
     *******************************************************************************
     * @brief getPort - return server start port
     *
     *  <1> Parameter Description:
     *  @param [In]  - void
     *  @return        void
     *
     *  <2> Detailed Description:
     *
     *******************************************************************************
     */
    int32_t getPort();

    /**
     *******************************************************************************
     * @brief startServer - start server
     *
     *  <1> Parameter Description:
     *  @param [In]  - void
     *  @return        void
     *
     *  <2> Detailed Description:
     *  this function will block programe, if you don't want it block programe, push
     *  it into sub thread.
     *******************************************************************************
     */
    void startServer();

    /**
     *******************************************************************************
     * @brief stopServer - stop server
     *
     *  <1> Parameter Description:
     *  @param [In]  - void
     *  @return        void
     *
     *  <2> Detailed Description:
     *  this function stop server work, if call this function, server will not receive
     *  message any more.
     *******************************************************************************
     */
    void stopServer();

    /**
     *******************************************************************************
     * @brief notifyUser - notify user to connect server
     *
     *  <1> Parameter Description:
     *  @param [In]  - std::string userIP, user IP
     *  @return        void
     *
     *  <2> Detailed Description:
     *  this function notify user to connect server, write server port in message body.
     *******************************************************************************
     */
    void notifyUser(std::string userIP);

    static uint32_t getModuleHandler(const uint32_t modCode,
                                     msgHandlerPtr_t& moduleHanderPtr);

private:
    void accept();

    int32_t port_;
    boost::asio::io_service io_service_;
    std::shared_ptr<tcp::acceptor> acceptorPtr_;
    tcp::socket socket_;
    sessionMap_t sessionMap_;

    static pool threadPool_;
    static map<uint32_t, msgHandlerPtr_t> registerMap_;
    static std::atomic<uint64_t> msgIdIndex_;

};


}

#endif

