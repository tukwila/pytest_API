/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   moduleMaskCode.h
 * @brief  All project module code define
 *******************************************************************************
 */

#ifndef MODULE_MASK_CODE_H
#define MODULE_MASK_CODE_H

#include "typeDef.h"


namespace roadDBCore
{

//success return value
const uint32_t ROAD_DATABASE_OK = 0;
const uint32_t ROAD_DATABASE_FAIL = -1;

//MODULE MASK CODE
const uint32_t COMMON_MASK_CODE  = 0X10000000;
const uint32_t VEHICLE_MASK_CODE = 0X20000000;
const uint32_t SERVER_MASK_CODE  = 0X30000000;
const uint32_t TOOL_MASK_CODE    = 0X40000000;

//COMMON_SUB_MODULE_MASK_CODE
const uint32_t COMMON_UTILITY_FUNS  = COMMON_MASK_CODE + 0X0100000;
const uint32_t COMMON_SERIALIZATION = COMMON_MASK_CODE + 0X0200000;
const uint32_t COMMON_SEGMENT       = COMMON_MASK_CODE + 0X0300000;
const uint32_t COMMON_JSON          = COMMON_MASK_CODE + 0X0400000;
const uint32_t COMMON_LIB_MESSAGE   = COMMON_MASK_CODE + 0X0500000;

//VEHICLE_SUB_MODULE_MASK_CODE
const uint32_t VEHICLE_ALGO_COMMOM       = VEHICLE_MASK_CODE + 0X0100000;
const uint32_t VEHICLE_ALGO_DENSE_SLAM   = VEHICLE_MASK_CODE + 0X0200000;
const uint32_t VEHICLE_ALGO_SPARSE_SLAM  = VEHICLE_MASK_CODE + 0X0300000;
const uint32_t VEHICLE_ALGO_LOCALIZATION = VEHICLE_MASK_CODE + 0X0400000;
const uint32_t VEHICLE_ALGO_ROAD         = VEHICLE_MASK_CODE + 0X0500000;
const uint32_t VEHICLE_ALGO_SIGN         = VEHICLE_MASK_CODE + 0X0600000;
const uint32_t VEHICLE_SYSTEM            = VEHICLE_MASK_CODE + 0X0700000;
const uint32_t VEHICLE_FRAMEWORK         = VEHICLE_MASK_CODE + 0X0800000;
const uint32_t VEHICLE_DOWNLOADER        = VEHICLE_MASK_CODE + 0X0A00000;
const uint32_t VEHICLE_UPLOADER          = VEHICLE_MASK_CODE + 0X0B00000;
const uint32_t VEHICLE_ALGO_SVM          = VEHICLE_MASK_CODE + 0X0C00000;

//SERVER_SUB_MOUDLE_MASK_CODE
const uint32_t SERVER_ALGO                  = SERVER_MASK_CODE + 0X0100000;
const uint32_t SERVER_LANE_LINE_MERGING     = SERVER_MASK_CODE + 0X0200000;
const uint32_t SERVER_LANE_LOGIC_EXTRACTION = SERVER_MASK_CODE + 0X0300000;
const uint32_t SERVER_SYSTEM                = SERVER_MASK_CODE + 0X0400000;
const uint32_t SERVER_DB                    = SERVER_MASK_CODE + 0X0500000;
const uint32_t SERVER_STORM                 = SERVER_MASK_CODE + 0X0600000;
}


#endif

