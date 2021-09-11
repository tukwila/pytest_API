/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   vehicleLocErrorCode.h
 * @brief  vehicle localization error code define.
 *******************************************************************************
 */
#ifndef VEHICLE_LOC_ERROR_CODE_H
#define VEHICLE_LOC_ERROR_CODE_H

#include "typeDef.h"
#include "errorCode/moduleMaskCode.h"

enum SystemErrcode_E
{
    FAIL                   = -1,
    SUCCESS                = 0,
    ERR_SLAM_SYSTEM_E      = 0x01000000,
    ERR_PARA_PARSER_E      = 0x02000000,
    ERR_DATABASE_E         = 0x03000000,
    ERR_FRAME_E            = 0x04000000,
    ERR_FRAME_MANAGER_E    = 0x05000000,
    ERR_FEATURE_E          = 0x06000000,
    ERR_FEATURE_MANAGER_E  = 0x07000000,
    ERR_FEATURE_TRACKER_E  = 0x08000000,
    ERR_DB_TRACKER_E       = 0x09000000,
    ERR_TRIANGULATE_E      = 0x0A000000
};
namespace roadDBCore
{

//msckf
const uint32_t MSCKF_STATE_ERROR_E                  = VEHICLE_ALGO_LOCALIZATION + 0x1;

const uint32_t MSCKF_STATE_PREDICT_SUCCESS_E        = VEHICLE_ALGO_LOCALIZATION + 0x2;
const uint32_t MSCKF_STATE_2DUPDATE_SUCCESS_E       = VEHICLE_ALGO_LOCALIZATION + 0x3;
const uint32_t MSCKF_STATE_3DUPDATE_SUCCESS_E       = VEHICLE_ALGO_LOCALIZATION + 0x4;

const uint32_t MSCKF_STATE_PRECOV_NOT_SQUARE_E      = VEHICLE_ALGO_LOCALIZATION + 0x5;
const uint32_t MSCKF_STATE_PRECOV_DIM_ERROR_E       = VEHICLE_ALGO_LOCALIZATION + 0x6;

const uint32_t MSCKF_STATE_2DCOV_DIM_ERROR_E        = VEHICLE_ALGO_LOCALIZATION + 0x7;
const uint32_t MSCKF_STATE_2DCOV_NO_VALID_POINTS_E  = VEHICLE_ALGO_LOCALIZATION + 0x8;
const uint32_t MSCKF_STATE_SKIP_2DFRAME_E           = VEHICLE_ALGO_LOCALIZATION + 0x9;

const uint32_t MSCKF_STATE_3DCOV_DIM_ERROR_E        = VEHICLE_ALGO_LOCALIZATION + 0xA;
const uint32_t MSCKF_STATE_3DCOV_NO_VALID_POINTS_E  = VEHICLE_ALGO_LOCALIZATION + 0xB;
const uint32_t MSCKF_STATE_SKIP_3DFRAME_E           = VEHICLE_ALGO_LOCALIZATION + 0xC;

const uint32_t MSCKF_STEP_ERROR                     = VEHICLE_ALGO_LOCALIZATION + 0xD;
const uint32_t MSCKF_IMU_ERROR                      = VEHICLE_ALGO_LOCALIZATION + 0xE;

// error code for input data processor
const uint32_t INPUT_DATA_PROCESSOR                 = VEHICLE_ALGO_LOCALIZATION + 0X0001000;

const uint32_t INPUT_IMGMISS_OVER_THRESHOLD         = INPUT_DATA_PROCESSOR + 0X01;
const uint32_t INPUT_IMG_SEQUENCE_ERROR             = INPUT_DATA_PROCESSOR + 0X02;
const uint32_t INPUT_IMUMISS_OVER_THRESHOLD         = INPUT_DATA_PROCESSOR + 0X03;
const uint32_t INPUT_CHECK_FAILED                   = INPUT_DATA_PROCESSOR + 0X04;
const uint32_t INPUT_OBDMISS_OVER_THRESHOLD         = INPUT_DATA_PROCESSOR + 0X05;

const uint32_t INPUT_GPSDATA_NOT_READY              = INPUT_DATA_PROCESSOR + 0X11;
const uint32_t INPUT_IMUDATA_NOT_READY              = INPUT_DATA_PROCESSOR + 0X12;

const uint32_t INPUT_NO_IMU_TO_ALIGN                = INPUT_DATA_PROCESSOR + 0X21;
const uint32_t INPUT_IMU_SEQUENCE_ERROR             = INPUT_DATA_PROCESSOR + 0X22;
const uint32_t INPUT_IMU_NOT_CLOSE_ENOUGH           = INPUT_DATA_PROCESSOR + 0X23;
const uint32_t INPUT_NO_GPS_TO_ALIGN                = INPUT_DATA_PROCESSOR + 0X24;
const uint32_t INPUT_GPS_SEQUENCE_ERROR             = INPUT_DATA_PROCESSOR + 0X25;
const uint32_t INPUT_NO_OBD_TO_ALIGN                = INPUT_DATA_PROCESSOR + 0X26;
const uint32_t INPUT_OBD_SEQUENCE_ERROR             = INPUT_DATA_PROCESSOR + 0X27;

const uint32_t INPUT_IMUGPS_FUSION_FAILED           = INPUT_DATA_PROCESSOR + 0X31;

const uint32_t INPUT_IMG_DECODE_FAILED              = INPUT_DATA_PROCESSOR + 0X41;
const uint32_t INPUT_IMG_EMPTY                      = INPUT_DATA_PROCESSOR + 0X42;
const uint32_t INPUT_IMG_SIZE_LESS_THRD             = INPUT_DATA_PROCESSOR + 0X43;

// error code for db loader
const uint32_t DB_LOADER_ERROR_BASE                 = VEHICLE_ALGO_LOCALIZATION + 0X0002000;
const uint32_t DBLOADER_DIVISION_NOT_FOUND          = DB_LOADER_ERROR_BASE + 0x01;
const uint32_t DBLOADER_DB_PATH_EMPTY               = DB_LOADER_ERROR_BASE + 0x02;
const uint32_t DBLOADER_DB_PATH_NOT_EXIST           = DB_LOADER_ERROR_BASE + 0x03;
const uint32_t DBLOADER_INIT_GPS_CONVERTER_FAIL     = DB_LOADER_ERROR_BASE + 0x04;
const uint32_t DBLOADER_MORE_THAN_ONE_REFERENCE     = DB_LOADER_ERROR_BASE + 0x05;
const uint32_t DBLOADER_ROADDB_SERIALIZE_FAIL       = DB_LOADER_ERROR_BASE + 0x06;
const uint32_t DBLOADER_BOOST_SERIALIZE_FAIL        = DB_LOADER_ERROR_BASE + 0x07;
const uint32_t DBLOADER_STD_SERIALIZE_FAIL          = DB_LOADER_ERROR_BASE + 0x08;
const uint32_t DBLOADER_ERR_DB_TYPE                 = DB_LOADER_ERROR_BASE + 0x09;
const uint32_t DBLOADER_INVALID_GPS                 = DB_LOADER_ERROR_BASE + 0xA;
const uint32_t DBLOADER_INVALID_ANCHOR_GPS          = DB_LOADER_ERROR_BASE + 0xB;
const uint32_t DBLOADER_INVALID_DATALOADER          = DB_LOADER_ERROR_BASE + 0xC;

// error code for vehicle db API
const uint32_t RIV_VEHICLE_ERROR_BASE              = VEHICLE_ALGO_LOCALIZATION + 0X0003000;


// error code for outlier
const uint32_t OUTLIER_STATE_ERROR_BASE            = VEHICLE_ALGO_LOCALIZATION + 0X0004000;
const uint32_t OUTLIER_STATE_INLIER                = OUTLIER_STATE_ERROR_BASE + 0x01;
const uint32_t OUTLIER_STATE_OUTLIER               = OUTLIER_STATE_ERROR_BASE + 0x02;
const uint32_t OUTLIER_STATE_CANDIDATE             = OUTLIER_STATE_ERROR_BASE + 0x03;
}

#endif

