/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018-2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   EventCommon.h
 * @brief  Definition of common event data structures
 *******************************************************************************
 */


#include <memory>                  // std::shared_ptr
#include "typeDef.h"
#include "LogWrapper/LogWrapper.h" // log


#ifndef EVENT_COMMON
#define EVENT_COMMON

namespace roadDBCore
{
using EventID_t = uint32_t;

// Module Mask Code for Event ID
const EventID_t EVENT_ID_COM_BASE            = 0X10000000;
const EventID_t EVENT_ID_SERVER_BASE         = 0X20000000;
const EventID_t EVENT_ID_LOC_BASE            = 0X30000000;

const EventID_t EVENT_ID_COM_OVERFLOW        = EVENT_ID_COM_BASE + 1;

enum EVENT_TYPE_E: uint8_t
{
    EVENT_TYPE_MAX_E
};

enum EVENT_STATE_E: uint8_t
{
    EVENT_STATE_INIT_E = 0,
    EVENT_STATE_PROCESSING_E,
    EVENT_STATE_DONE_E,
    EVENT_STATE_IGNORE_E,
    EVENT_STATE_MAX_E
};

struct EventBase_t
{
    EventID_t id;
    EVENT_TYPE_E type;
    EVENT_STATE_E state;
    std::string info;

    EventBase_t(EventID_t idIn = 0,
                std::string infoIn = "",
                    EVENT_TYPE_E typeIn = EVENT_TYPE_MAX_E):
                    id(idIn),
                    type(typeIn),
                    state(EVENT_STATE_INIT_E),
                    info(infoIn)
    {}

    virtual std::string toString() const
    {
        return std::string("BaseEvent, ID:") + std::to_string(id);
    }
};

struct OverflowEvent_t: public EventBase_t
{
    OverflowEvent_t():EventBase_t(EVENT_ID_COM_OVERFLOW) {}

    std::string toString() const
    {
        return std::string("OverflowEvent, ID:") + std::to_string(EVENT_ID_COM_OVERFLOW);
    }
};


} // namespace roadDBCore




#endif // EVENT_COMMON




