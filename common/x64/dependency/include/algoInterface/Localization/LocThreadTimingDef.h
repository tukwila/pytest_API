/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018-2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   LocThreadTimingDef.h
 * @brief  Definition of thread timing for localization
 *******************************************************************************
 */
#pragma once


#include "threadTiming/threadTiming.h"




namespace algo
{
namespace vehicle
{
enum LOC_THREAD_E: uint8_t
{
    LOC_THREAD_MAIN_E = 1,
    LOC_THREAD_DATA_PRO_E = 2,
    LOC_THREAD_3D_MATCH_E = 3,
    LOC_THREAD_LOC_E = 4,
    LOC_THREAD_TRIANG_E = 5,
    LOC_THREAD_RIV_API_E = 6,
    LOC_THREAD_DR_GPS_E = 7,
    LOC_THREAD_DR_IMU_E = 8,
    LOC_THREAD_DR_IMAGE_E = 9,
    LOC_THREAD_POS_REPORT = 10,
    LOC_THREAD_MAX_E
};

enum MAIN_MOD_E: uint8_t
{
    MAIN_MOD_PARSE_CONFIG_E = 1,
    MAIN_MOD_LOAD_DB_E = 2,
    MAIN_MOD_DATA_PRO_E = 3,
    MAIN_MOD_3D_MATCH_E = 4,
    MAIN_MOD_LOC_E = 5,
    MAIN_MOD_DATA_RECV_E = 6,
    MAIN_MOD_MAX_E
};

enum DP_MOD_E: uint8_t
{
    DP_MOD_READ_RAW_DATA_E = 1,
    DP_MOD_RAW_DATA_PRE_E = 2,
    DP_MOD_IMG_DECODE_E = 3,
    DP_MOD_IMG_RESCALE_E = 4,
    DP_MOD_IMG_UNDISTORT_E = 5,
    DP_MOD_PACK_RAW_DATA_E = 6,
    DP_MOD_SEARCH_DB_E = 7,
    DP_MOD_MAX_E
};

enum MATCH3D_MOD_E: uint8_t
{
    MATCH3D_MOD_STOP_DETECT_E = 1,
    MATCH3D_MOD_LANEKEEP_DETECT_E = 2,
    MATCH3D_MOD_LANEKEEP_MATCH_E = 3,
    MATCH3D_MOD_2D_EXTRACT_E = 4,
    MATCH3D_MOD_REPROJECTION_E = 5,
    MATCH3D_MOD_BRUTE_FORCE_E = 6,
    MATCH3D_MOD_PNP_E = 7,
    MATCH3D_MOD_PROCESS_END_E = 8,
    MATCH3D_MOD_MAX_E
};

enum LOC_MOD_E: uint8_t
{
    LOC_MOD_TRACK_E = 1,
    LOC_MOD_MSCKF_E = 2,
    LOC_MOD_INIT_MGR_E = 3,
    LOC_MOD_INIT_MSCKF_E = 4,
    LOC_MOD_INIT_TRACK_E = 5,
    LOC_MOD_MSCKF_DIS_E = 6,
    LOC_MOD_MAX_E
};

enum TRIANG_MOD_E: uint8_t
{
    TRIANG_MOD_INIT_MGR_E = 1,
    TRIANG_MOD_PREDICT_E = 2,
    TRIANG_MOD_ADD_OBJ_E = 3,
    TRIANG_MOD_BUILD_POINT_E = 4,
    TRIANG_MOD_BA_E = 5,
    TRIANG_MOD_MAX_E
};

inline void locTiming(uint8_t thdId, uint8_t mainMod, uint16_t subMod, int32_t frameId = -1) {roadDBCore::timingNow(thdId, mainMod, subMod, frameId);}

inline void locTimingStart(uint8_t thdId, int32_t frameId = -1) {roadDBCore::timingStart(thdId, frameId);}

inline void locTimingEnd(uint8_t thdId, int32_t frameId = -1) {roadDBCore::timingEnd(thdId, frameId);}

inline void locTimingDump(const std::string &fileName = "./threadTiming.bin") {roadDBCore::timingDump(fileName);}


} // namespace algo

} // namespace vehicle






