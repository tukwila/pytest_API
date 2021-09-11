/**
*******************************************************************************
*                       Continental Confidential
*                  Copyright (c) Continental AG. 2018-2019
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
******************************************************************************
* @file   ImuTagger.h
* @brief  Declaration of ImuTagger
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
#include "CommUtils.h"

using namespace libSensor;

class ImuTagger
{
public:
    ImuTagger(const std::string & imuFile);
    bool init(const std::vector<uint64_t> & timeInfo);
    bool getData(uint64_t starttime, uint64_t endtime, std::vector<Imu9DofData> & data);
    bool getValidTagIndex(uint64_t & start, uint64_t & end);
    void uninit();
    virtual ~ImuTagger();
private:
    bool doTag(const std::vector<uint64_t> & timeInfo);
    inline bool getTwoRefLocViaTime(const uint64_t & time, std::vector<Imu9DofData>gpsRecords, Imu9DofData l1, Imu9DofData l2, size_t * lastFindIndex);
    void linearInterpolation(const Imu9DofData & l1, const Imu9DofData & l2, Imu9DofData & l);
    std::string imuFileStr_;
    uint64_t imuTaggedFrameNum_;
    uint64_t imuTaggedStartIndex_;
    std::shared_ptr<SensorFormatStreamReader<Imu9DofData>> imuFileReader_;
    std::vector<Imu9DofData> taggedImuRecords_;
    bool tagged_;
};
