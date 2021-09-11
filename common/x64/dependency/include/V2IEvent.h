/**
 *******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB AG. 2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 ******************************************************************************
 * @file   V2IEvent.h
 * @brief  Interface of V2IEvent 
 *******************************************************************************
 */

#ifndef V2IEvent_HPP
#define V2IEvent_HPP
#include "jsoncpp.hpp"
#include <string>
#include <memory>
typedef enum {
    V2IEVENT_DEVICE_REG_E = 1,
    V2IEVENT_DEVICE_ONLINE_NOTI_E = 2, 
    V2IEVENT_DEVICE_ERROR_NOTI_E = 3, 
    V2IEVENT_DEVICE_UPDATE_NOTI_E = 4, 
    V2IEVENT_DEVICE_RTV_REPORT_E = 9, 
}V2IEVENT_E;




class V2IEvent;
typedef std::shared_ptr<V2IEvent> V2IEventPtr; 

class V2IEvent 
{
    public:

        virtual ~V2IEvent();
        virtual std::string makeEvent(Json::Value body) = 0;
        virtual bool sendEvent() = 0;


        friend class V2ICreater;
    protected:
        V2IEvent();

};


class V2ICreater
{
    public :
        V2IEventPtr operator()(V2IEVENT_E eventType);
};


#endif
