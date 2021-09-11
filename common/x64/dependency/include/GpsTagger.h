/**
*******************************************************************************
*                       Continental Confidential
*                  Copyright (c) Continental AG. 2018-2019
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
******************************************************************************
* @file   GpsTagger.h
* @brief  Declaration of GpsTagger
*******************************************************************************
*/
#pragma once

#include <stdint.h>
#include <string>
#include <vector>
#include <memory>

#include "sensorDataType.h"
#include "RoadDBVideoFileReader.h"
#include "sensorFormatStreamReader.h"

using namespace libSensor;

class GpsTagger
{
public:
    GpsTagger(const std::string & gpsFile);
    bool init(const std::vector<uint64_t> & timeInfo);
    bool getData(uint32_t fStart, uint32_t fEnd, std::vector<GpsData> & data);
    bool getData(uint32_t index, GpsData & data);
    bool getValidTagIndex(uint64_t & start, uint64_t & end);
    void uninit();
    virtual ~GpsTagger();
private:
    bool doTag(const std::vector<uint64_t> & timeInfo);
    inline bool getTwoRefLocViaTime(const uint64_t & time, std::vector<GpsData> & gpsRecords, GpsData & l1, GpsData & l2, size_t * lastFindIndex);
    void linearInterpolation(const GpsData & l1, const GpsData & l2, GpsData & l);
    std::string gpsFileStr_;
    uint64_t gpsTaggedFrameNum_;
    uint64_t gpsTaggedStartIndex_;
    std::shared_ptr<SensorFormatStreamReader<GpsData>> gpsFileReader_;
    std::vector<GpsData> taggedGpsRecords_;
    bool tagged_;
};
