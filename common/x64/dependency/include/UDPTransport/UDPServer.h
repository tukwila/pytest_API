/**
 *******************************************************************************
 *                         RoadDB Confidential
 *           Copyright (c) Continental AG. 2016, RoadDB 2016-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file    UDPServer.h
 * @brief   UDP server class
 *
 * Change Log:
 *      Date             Who                      What
 *      2018.05.17       <Dexun.liu>               Created
 *******************************************************************************
 */

#ifndef LOC_AGENT_UDP_SERVER__H__
#define LOC_AGENT_UDP_SERVER__H__

#include <string>
#include <boost/asio.hpp>
#include <iostream>
#include <atomic>
#include <boost/thread.hpp>
#include <functional>
#include <set>
#include "UDPMsgSerialization/DrawMessage.h"

using namespace boost::asio;

namespace agent
{

using namespace roadDBCore::rdbSerialization;
typedef std::function<void(const std::string&)> func_t;
typedef std::function<void (std::shared_ptr<MessageObject_t>)> payload_fun_t;


//class udpServer.
class udpServer
{
public:

	udpServer();
	virtual ~udpServer();

/**************************************************************************//**
 @Function
uint32_t udpServer::init(std::string &ip)

 @Description   init function of UDPServer

 @Param[in]     ip - string

 @Return        uint32_t

 @Cautions      None.
*//***************************************************************************/
	uint32_t init(std::string &ip, int iPort = 2323);
/**************************************************************************//**
 @Function  void udpServer::start()

 @Description   start UDPServer

 @Param        none

 @Return       void

 @Cautions      None.
*//***************************************************************************/
	void start();

/**************************************************************************//**
 @Function  void udpServer::stop()

 @Description   stop UDPServer

 @Param        none

 @Return       void

 @Cautions      None.
*//***************************************************************************/
	void stop();
	void setdataStringFunc(func_t &fun);
	void setAsyncRecv();

	void join();

private: 
    void reCombinePacket(char* buffer, uint32_t len);
	uint32_t checkHeader(const PackageHeader& head);
private:

	std::atomic<bool> isRuning_;
	std::shared_ptr<boost::thread> threadPtr_;

	std::map<uint64_t, std::string> mapPacket_; //packet index, packet context.
	std::map<uint64_t, std::set<uint32_t>> mapIndex_;  //packet index, packet sub index.

	boost::asio::io_service ioService_;
	std::shared_ptr<ip::udp::socket> socketPtr_;
	func_t func_;

	ip::udp::endpoint romote_endpoint;
	char buffer[MAX_LENGTH];
};

class udpServerAdapter : public udpServer
{
public:
	udpServerAdapter();
	virtual ~udpServerAdapter();
	
	void setPayloadFunc(payload_fun_t fun, func_t func_str = nullptr);

private:
	void setData(const std::string &str);
	payload_fun_t loc_func;
	func_t func_str_;
};

}//!agent


#endif //!LOC_AGENT_UDP_SERVER__H__
