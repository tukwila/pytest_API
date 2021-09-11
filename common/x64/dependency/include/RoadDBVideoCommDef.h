/**
*******************************************************************************
*                       Continental Confidential
*                  Copyright (c) Continental AG. 2018-2019
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
******************************************************************************
* @file   RoadDBVideoCommDef.h
* @brief  Implementation of RoadDBVideoCommDef 
*******************************************************************************
*/
#ifndef ROADDBVIDEOCOMMDEF_H
#define ROADDBVIDEOCOMMDEF_H 
#include <string>

enum ROAD_DB_VERSION_E 
{
    ROAD_DB_VERSION_ZERO_ONE_E,
    ROAD_DB_VERSION_ZERO_TOW_E,
    ROAD_DB_VERSION_ZERO_THREE_E,
    ROAD_DB_VERSION_ZERO_FOUR_E,
    ROAD_DB_VERSION_ONE_ZERO_E,
    ROAD_DB_VERSION_MAX_E
};

enum ROAD_DB_CODEC_E : uint8_t
{
    ROAD_DB_CODEC_H264_E = 0,
    ROAD_DB_CODEC_JPG_E,
    ROAD_DB_CODEC_MAX_E
};

enum ROAD_DB_FORMAT_E : uint8_t 
{
    ROAD_DB_FORMAT_YUV420_E = 0,
    ROAD_DB_FORMAT_RGB_E,
    ROAD_DB_FORMAT_BGR_E,
    ROAD_DB_FORMAT_GRAY_E,
    ROAD_DB_FORMAT_MAX_E
};


enum SCALE_FACTOR_E 
{
    SCALE_FACTOR_NO_E = 0,
    SCALE_FACTOR_7_8_E,
    SCALE_FACTOR_3_4_E,
    SCALE_FACTOR_5_8_E,
    SCALE_FACTOR_1_2_E,
    SCALE_FACTOR_3_8_E,
    SCALE_FACTOR_1_4_E,
    SCALE_FACTOR_1_8_E,
    SCALE_FACTOR_MAX_E
};



struct Resolution
{
    uint16_t width;
    uint16_t height;
};

std::string roaddbVersion2String(ROAD_DB_VERSION_E eVersion);
ROAD_DB_VERSION_E string2RoaddbVersion(std::string version);


std::string roaddbCodec2String(ROAD_DB_CODEC_E eCodec);
ROAD_DB_CODEC_E string2RoaddbCodec(std::string codec);


std::string roaddbFormat2String(ROAD_DB_FORMAT_E eFormat);
ROAD_DB_FORMAT_E string2RroaddbFormat(std::string format);


#endif
