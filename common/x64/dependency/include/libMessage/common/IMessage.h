/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   IMessage.h
 * @brief
 *******************************************************************************
 */


#ifndef IMESSAGE_H
#define IMESSAGE_H


namespace roadDBCore
{

class IMessage
{
public:
    virtual uint64_t getMsgId() = 0;
    virtual uint32_t getModCode() = 0;
    virtual char* getMsgBody() = 0;
    virtual uint32_t getMsgBodyLength() = 0;
    virtual uint32_t getMsgResult() = 0;
};


}

#endif


