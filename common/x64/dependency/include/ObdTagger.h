/**
*******************************************************************************
*                       Continental Confidential
*                  Copyright (c) Continental AG. 2018-2019
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
******************************************************************************
* @file   OBDTagger.h
* @brief  Declaration of OBDTagger
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

class OBDTagger
{
public:
    OBDTagger(const std::string & gpsFile);
    bool init(const std::vector<uint64_t> & timeInfo);
    bool getData(uint32_t fStart, uint32_t fEnd, std::vector<OBDData> & data);
    bool getData(uint32_t index, OBDData & data);
    bool getValidTagIndex(uint64_t & start, uint64_t & end);
    bool doTag(const std::vector<uint64_t> & timeInfo);
    void uninit();
    virtual ~OBDTagger();
private:
    inline bool getTwoRefLocViaTime(const uint64_t & time, std::vector<OBDData> & obdRecords, OBDData & l1, OBDData & l2, size_t * lastFindIndex);
    void linearInterpolation(const OBDData & l1, const OBDData & l2, OBDData & l);
    std::string obdFileStr_;
    uint64_t obdTaggedFrameNum_;
    uint64_t obdTaggedStartIndex_;
    std::shared_ptr<SensorFormatStreamReader<OBDData>> obdFileReader_;
    std::vector<OBDData> taggedOBDRecords_;
    bool tagged_;
};
