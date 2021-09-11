/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   utilityData.h
 * @brief  utility functions
 *******************************************************************************
 */
#ifndef _UTILITY_DATA_H_
#define _UTILITY_DATA_H_

#include <sstream>
#include <iomanip>
#include "typeDef.h"
#include "LogWrapper/LogWrapper.h"

namespace roadDBCore
{

//the unique id of a video, the Index and  the KeyFrame file, the trafficSign file , the roadGeometry is one-to-one correspondence
struct Index
{
    std::string   vehicleID;  //vehicle ID
    uint64_t      timeStamp;  //the time stamp of video
    int32_t       tileID;      //the tile id of video start from.

    Index() : vehicleID(""), timeStamp(0), tileID(-1) {}

    bool operator == (const Index &idx)
    {
        if ((vehicleID == idx.vehicleID) && (timeStamp == idx.timeStamp) && (tileID == idx.tileID))
        {
            return true;
        }

        return false;
    }

    bool operator != (const Index &idx)
    {
        return !(*this==idx);
    }

    Index& operator = (const Index &idx)
    {
        vehicleID = idx.vehicleID;
        timeStamp = idx.timeStamp;
        tileID = idx.tileID;

        return *this;
    }
};

//extend StringStreamX  for set precision
class StringStreamX: public std::stringstream
{
public:
    StringStreamX(const int32_t& precision = 16, const bool bScientific = true)
    {
        if (!bScientific)
        {
            *this << std::setiosflags(std::ios::fixed);
        }

        *this << std::setprecision(precision);
    }
};


}

#endif

