/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017-2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   MessageClient.h
 * @brief  class MessageClient.
 *******************************************************************************
 */

#ifndef MESSAGE_CLIENT_H
#define MESSAGE_CLIENT_H

#include <boost/asio.hpp>
#include "libMessage/client/message_base.h"
#include "libMessage/client/IMessageClient.h"


namespace roadDBCore
{

class MessageClient : public IMessageClient
{
public:
    MessageClient(std::pair<std::string, std::string> &sp, std::string localPort = "");
//    static MessageClient* instance(std::string &serverIP, std::string &serverPort,
//            std::string localPort = "");
    virtual ~MessageClient();

    /*send msg with type of MSG_TYPE_SYNC_E*/
    uint32_t syncSend(char* data, const int32_t dataLength, const uint32_t modCode,
                std::shared_ptr<IMessage> &rspMsg);
    uint32_t syncSend_Copy(char* data, const int32_t dataLength, const uint32_t modCode,
                std::shared_ptr<IMessage> &rspMsg);

    /*send msg with type of MSG_TYPE_NORET_SYNC_E, server won't respond*/
    uint32_t syncSend_NoRsp(char* data, const int32_t dataLength, const uint32_t modCode);
    uint32_t syncSend_Copy_NoRsp(char* data, const int32_t dataLength, const uint32_t modCode);

    /*send msg with type of MSG_TYPE_ASYNC_E; set msgId back to user*/
    uint32_t asyncSend(char* data, const int32_t dataLength, const uint32_t modCode,
            msgCbFunc cb, uint64_t &msgId);
    uint32_t asyncSend_Copy(char* data, const int32_t dataLength, const uint32_t modCode,
            msgCbFunc cb, uint64_t &msgId);

    /*send msg with type of MSG_TYPE_NORET_ASYNC_E, server won't respond; set msgId back to user*/
    uint32_t asyncSend_NoRsp(char* data, const int32_t dataLength, const uint32_t modCode,
            msgCbFunc cb, uint64_t &msgId);
    uint32_t asyncSend_Copy_NoRsp(char* data, const int32_t dataLength, const uint32_t modCode,
            msgCbFunc cb, uint64_t &msgId);

    uint32_t registerSvrMsgCallback(const uint32_t modCode, msgCbFunc cb);

    int32_t getInitStatus();
    bool isConnected();
    uint32_t start();
    void stop();

protected:
//    MessageClient(std::pair<std::string, std::string> &sp, std::string &localPort);

private:
    uint32_t init();
    void release();
    void closeSocket();
    void close_socket();

    void connect();
    void reconnect();
    void restartSocket();
    void asyncConnect();
    void asyncBindConnect();

    void startCnxtThread();

    uint32_t syncSend(std::shared_ptr<MessageObj> msg);
    uint32_t syncSend(std::shared_ptr<MessageObj> msg, std::shared_ptr<IMessage> &rspMsg);
    uint32_t asyncSend(std::shared_ptr<MessageObj> msg, msgCbFunc cb);

    uint32_t postMsg2Service(std::shared_ptr<MessageObj> msg, msgCbFunc cb);
    void stopSyncWaiting();
    void asyncSend2Socket();

    void writeMsgHeader(std::shared_ptr<MessageObj> msg);
    void writeMsgBody(std::shared_ptr<MessageObj> msg);
    void sendAck(std::shared_ptr<MessageObj> msg, uint32_t result);

    void readMsgHeader();
    void readMsgBody(const msgHeader_t& msg);
    std::shared_ptr<MessageObj> getSyncRspMsg(const uint64_t msgId);

    uint32_t startRcvMsgThread();
    void rcvMsgRun(modContext_t &handleRcvMsgCnxt);

    void modThdRun(std::shared_ptr<modContext_t> modCnxt);
    void handleModMsg(std::shared_ptr<modContext_t> modCnxt);

private:
    enum IOSERVICE_STA_E
    {
        IOSERVICE_STA_STOPPED = 0,
        IOSERVICE_STA_STARTING = 1,
        IOSERVICE_STA_RUNNING = 2
    };

    // status of client: 0 : not inited, 1: inited
    volatile int32_t initStatus_;
    // protect initStatus_
    std::mutex init_mtx_;

    // it changes from io_service(boost 1.60) to io_context(boost 1.65)
    // as we use boost 1.60, replacing io_context by io_service
    boost::asio::io_service io_service_;
    // to ensure io_service not to exit before being connected
    std::shared_ptr<boost::asio::io_service::work> io_work_;
    // flag to stop io service
    volatile bool stopIOService_;

    boost::asio::ip::tcp::socket socket_;

    // tcp::resolver::results_type for boost 1.65
    // tcp::resolver::iterator for boost 1.60
    boost::asio::ip::tcp::resolver::iterator endpoints_it_;

    // status of socket connection: 0 : not connected, 1: connecting, 2: connected
    volatile CONNT_STA_E connectStatus_;
    // socket age to indicate the reconnection times.
    volatile int32_t socketAge_;

    // ms for socket reconnection times
    int32_t socket_reconnect_times_;

    std::thread ioServiceThread_;
    volatile int32_t ioServiceThreadRun_;

    // thread to handle received msg from socket
    modContext_t handleRcvMsgCntx_;

    // queue of the responded msg of the sync msg
    msgQueue syncRspMsgQue_;

    // queue of the msg to write to socket
    msgQueue socketWriteMsgQue_;
    // is socket writing/sending or not
    volatile bool socketIsWriting_;

    // atomic msgId
    std::atomic<uint64_t> msgId_;

    // msgId : msgCBInfo_t
    std::map<uint64_t, std::shared_ptr<msgCBInfo_t>> mMsgCallback_;
    // protect mMsgCallback_
    std::mutex msgCb_mtx_;

    // <modCode : msgCbFunc>, for handling server sponsored msg, one callback for a module
    std::map<uint32_t, msgCbFunc> mSvrMsgCallback_;
    // protect mSvrMsgCallback_
    std::mutex svrMsgCb_mtx_;

    // <module Id : modContext_t>, for handling async msg, each module run in a distinct thread
    std::map<uint16_t, std::shared_ptr<modContext_t>> mModContext_;


    // /* constant values */
    // "serverIP", "port"
    const std::pair<std::string, std::string> sp_;

    // local socket port, set by user; if not set, assigned automatically by socket member function
    const int32_t localPort_;

    // ms for one loop waiting
//    const int32_t SINGLE_LOOP_MAX_TIME_OUT = 100;

    // longest time for CONNT_ACK_SOCKET waiting, ms
    const int32_t SOCKET_ACK_TIME_OUT = 15000; //15s

    // longest time for sync msg waiting, ms
    const int32_t SYNC_MAX_TIME_OUT = 60000; //60s

    // ms for socket reconnection waiting interval
    const int32_t SOCKET_RECONNECT_WAITING_INTERVAL = 2000;

    // ms for socket reconnection times
    const int32_t SOCKET_RECONNECT_MAX_TIMES = 3;

    // /* static values */
    // <"serverIP", "port"> : MessageClient
//    static std::map<std::pair<std::string, std::string>, MessageClient*> mSP2MessageClient_;
    // protect mSP2MessageClient_ only
//    static std::mutex mcHolder_mtx_;
};

}

#endif // MESSAGE_CLIENT_H
