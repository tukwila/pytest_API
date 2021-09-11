/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   vehicleSysErrorCode.h
 * @brief  vehicle system error code define.
 *******************************************************************************
 */
#ifndef VEHICLE_SYS_ERROR_CODE_H
#define VEHICLE_SYS_ERROR_CODE_H

#include "typeDef.h"
#include "errorCode/moduleMaskCode.h"

namespace roadDBCore
{

const uint32_t SCHEDULE_ERR_NOT_RUN                 = VEHICLE_SYSTEM + 0X1;
const uint32_t SCHEDULE_ERR_COMMON                  = VEHICLE_SYSTEM + 0X2;
const uint32_t SCHEDULE_ERR_ARGC                    = VEHICLE_SYSTEM + 0X3;
const uint32_t SCHEDULE_ERR_OPT_NOT_EXIST           = VEHICLE_SYSTEM + 0X4;
const uint32_t SCHEDULE_ERR_OPT_NULL                = VEHICLE_SYSTEM + 0X5;
const uint32_t SCHEDULE_ERR_PARAM_NULL              = VEHICLE_SYSTEM + 0X6;
const uint32_t SCHEDULE_ERR_PARAM_MAX_LEN           = VEHICLE_SYSTEM + 0X7;
const uint32_t SCHEDULE_ERR_VIDEO_NULL              = VEHICLE_SYSTEM + 0X8;
const uint32_t SCHEDULE_ERR_CAM_NULL                = VEHICLE_SYSTEM + 0X9;
const uint32_t SCHEDULE_ERR_ALG_NULL                = VEHICLE_SYSTEM + 0XA;
const uint32_t SCHEDULE_ERR_TEMP_NULL               = VEHICLE_SYSTEM + 0XB;
const uint32_t SCHEDULE_ERR_LOG_NULL                = VEHICLE_SYSTEM + 0XC;
const uint32_t SCHEDULE_ERR_DEBUG_NULL              = VEHICLE_SYSTEM + 0XD;
const uint32_t SCHEDULE_ERR_OUTFILE_NULL            = VEHICLE_SYSTEM + 0XE;
const uint32_t SCHEDULE_ERR_GENHEAD_FAIL            = VEHICLE_SYSTEM + 0XF;

const uint32_t DATA_PARSER_ERR_COM                  = VEHICLE_SYSTEM + 0X30;
const uint32_t DATA_PARSER_ERR_FILE_NAME            = VEHICLE_SYSTEM + 0X31;
const uint32_t DATA_PARSER_ERR_FILE_TYPE            = VEHICLE_SYSTEM + 0X32;
const uint32_t DATA_PARSER_ERR_DATA_TYPE            = VEHICLE_SYSTEM + 0X33;
const uint32_t DATA_PARSER_ERR_INVALID_PARAMETER    = VEHICLE_SYSTEM + 0X34;
const uint32_t DATA_PARSER_ERR_LAST_FRAME           = VEHICLE_SYSTEM + 0X35;
const uint32_t DATA_PARSER_ERR_NOT_INIT             = VEHICLE_SYSTEM + 0X36;
const uint32_t DATA_PARSER_ERR_DUPLICATE_INIT       = VEHICLE_SYSTEM + 0X37;
const uint32_t DATA_PARSER_ERR_VIDEO_FORMAT         = VEHICLE_SYSTEM + 0X38;
const uint32_t DATA_PARSER_ERR_MMAP_FAIL            = VEHICLE_SYSTEM + 0X39;
const uint32_t DATA_PARSER_ERR_OPEN_FAIL            = VEHICLE_SYSTEM + 0X3A;
const uint32_t DATA_PARSER_ERR_FILE_SIZE            = VEHICLE_SYSTEM + 0X3B;
const uint32_t DATA_PARSER_ERR_NULL_POINTER         = VEHICLE_SYSTEM + 0X3C;
const uint32_t DATA_PARSER_ERR_FILE_NOT_EXIST       = VEHICLE_SYSTEM + 0X3D;
const uint32_t DATA_PARSER_ERR_CANNOT_READ          = VEHICLE_SYSTEM + 0X3E;

const uint32_t COMMON_ERR_CREATE_FOLDER             = VEHICLE_SYSTEM + 0X60;
const uint32_t COMMON_ERR_ALLOCATE_MEMORY           = VEHICLE_SYSTEM + 0X61;
const uint32_t COMMON_ERR_PARAMETER                 = VEHICLE_SYSTEM + 0X62;
const uint32_t COMMON_ERR_INIT                      = VEHICLE_SYSTEM + 0X63;
const uint32_t COMMON_ERR_GET                       = VEHICLE_SYSTEM + 0X64;

const uint32_t ALGO_ADAPT_ERR_SETTING_FAILED        = VEHICLE_SYSTEM + 0X90;
const uint32_t ALGO_ADAPT_ERR_INIT_FAILED           = VEHICLE_SYSTEM + 0x91;
const uint32_t ALGO_ADAPT_ERR_TRACK_FAILED          = VEHICLE_SYSTEM + 0x92;
const uint32_t ALGO_ADAPT_ERR_PRO_FAILED            = VEHICLE_SYSTEM + 0x93;
const uint32_t ALGO_ADAPT_ERR_NULLPTR               = VEHICLE_SYSTEM + 0x94;
const uint32_t IMAGE_AND_GPS_ERR_INIT               = VEHICLE_SYSTEM + 0x95;
const uint32_t IMAGE_AND_GPS_ERR_PARAMETER          = VEHICLE_SYSTEM + 0x96;
}

#endif

