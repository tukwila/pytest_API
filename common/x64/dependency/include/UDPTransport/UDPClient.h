#ifndef LOC_AGENT_UDP_CLIENT__H__
#define LOC_AGENT_UDP_CLIENT__H__

#include <string>
#include <boost/asio.hpp>
#include <iostream>
#include <atomic>
#include <memory>
#include <mutex>
#include <boost/thread.hpp>
#include <boost/lockfree/queue.hpp>
#include "UDPMsgSerialization/SerialDrawMsg.h"
#include "UDPMsgSerialization/DrawMessage.h"

using namespace boost::asio;
class ImageData;

namespace algo{
    namespace vehicle{
        struct RawImageData_t;
    };
};

namespace agent
{

//class udpclient.
class UDPClient
{
public:
	UDPClient();
	virtual ~UDPClient();

	uint32_t init(const std::string &ip, uint16_t port);
	void close();
	uint32_t push(std::string &data);
    
    /**************************************************************************//**
     @Function      int sendMessage(roadDBCore::rdbSerialization::MessageObject_t vMsg)
    
     @Description   send message obj to the dest server 
    
     @Param[in]     vMsg - message obj to send
    
     @Return        0 for success
    
     @Cautions      the error info will print at stderr when none 0 returned.
    *//***************************************************************************/

    int sendMessage(std::shared_ptr<roadDBCore::rdbSerialization::MessageObject_t> vMsg);

	uint32_t getCurrentIndex();

public:
	uint32_t headLen;
    volatile bool bInitialized;
    std::string strIP;
    uint16_t uPort;

private:
    UDPClient(const UDPClient&);
    UDPClient& operator=(const UDPClient&);

	void splitAndSendPacket(const std::string &data);
    bool IsValidIpv4(const char *str);
	
private:

	boost::asio::io_service ioService_;
	std::shared_ptr<ip::udp::endpoint> remoteEndPtr_;
	std::shared_ptr<ip::udp::socket> socketPtr_;

	char *sendBuffer_;
	std::atomic_uint packetIndex_;
    boost::recursive_mutex recursiveMutex_;
	
};

}//!agent


#endif //!LOC_AGENT_UDP_CLIENT__H__
