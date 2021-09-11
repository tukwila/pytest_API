/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   rivErrCode.h
 * @brief  Definition of Error Code  
 *******************************************************************************
 */
#pragma once

namespace RDBVehicleAPI
{
typedef int32_t     RIVErrCode_t;
const RIVErrCode_t   RIV_COMN_SUCCESS                = 0;

const RIVErrCode_t   RIV_COMN_INITIALIZED            = 101;      //logic api already initialized

const RIVErrCode_t   RIV_COMN_GET_SEGID_BY_INITAL_POSTION_FAIL = 102;
const RIVErrCode_t   RIV_COMN_LOAD_DATA_FAIL         = 103;
const RIVErrCode_t   RIV_COMN_NO_INITAL_POSTION_WHILE_INITIALIZING = 104;
const RIVErrCode_t   RIV_COMN_INIT_SUMMARY_FAILED    = 105;
const RIVErrCode_t   RIV_COMN_FILE_NOT_EXIST         = 106;
const RIVErrCode_t   RIV_COMN_DATA_MISSING           = 107;

const RIVErrCode_t   RIV_COMN_PARAS_ERROR            = 201;      //input paras error
const RIVErrCode_t   RIV_COMN_INVALID_THREAD_ATTR    = 202;      //failed to initialize thread attributes
const RIVErrCode_t   RIV_COMN_INVALID_POSTION        = 203;      //invalid initial position
const RIVErrCode_t   RIV_COMN_INVALID_RANGE          = 204;      //invalid range
const RIVErrCode_t   RIV_COMN_INVALID_SEGMENT_ID     = 205;      //invalid segment id
const RIVErrCode_t   RIV_COMN_INVALID_REFRESH_PARAS  = 206;      //invalid refresh paras

const RIVErrCode_t   RIV_COMN_NO_SEGMENT_EXISTS_AROUND_POSITION = 301;  //No segment file exists around new reported gps postion
const RIVErrCode_t   RIV_COMN_SERVICE_STOPPED        = 302;      //
const RIVErrCode_t   RIV_COMN_SEGMENT_UNCHANGED      = 303;      //
const RIVErrCode_t   RIV_COMN_SAME_SEGMENT_WITH_LAST_POSITION = 304;
const RIVErrCode_t   RIV_COMN_MAX_REPORTED_GPS       = 305;
const RIVErrCode_t   RIV_COMN_UNSUPPORTED_CALL       = 306;
const RIVErrCode_t   RIV_COMN_CACHE_RANGE_UNCHANGE   = 307;

const RIVErrCode_t   RIV_COMN_GPS_ACCEPTED           = 401;
const RIVErrCode_t   RIV_COMN_GPS_CACHED             = 402;
const RIVErrCode_t   RIV_COMN_GPS_UNACCEPTED         = 403;

const RIVErrCode_t   RIV_COMN_COMMUNICATION_ERROR    = 1001;     //server not run
const RIVErrCode_t   RIV_COMN_VER_ERROR              = 1002;     //version error, cs used, not used now                    
const RIVErrCode_t   RIV_COMN_TIME_OUT_ERROR         = 1003;     //response time-out, you can send request again

const RIVErrCode_t   RIV_COMN_DBINIT_FAIL            = 10001;    //not used now
const RIVErrCode_t   RIV_COMN_CACHEINIT_FAIL         = 10002;    // cache data fail
const RIVErrCode_t   RIV_COMN_CACHEAGENT_INIT_FAIL   = 10003;    // cache agent start failed
  

const RIVErrCode_t   RIV_COMN_UNINITIALIZED          = 20001;    // need to call  mgr::init before getData
const RIVErrCode_t   RIV_COMN_INNER_ERROR            = 20002;    // inner error such as memory lack
const RIVErrCode_t   RIV_COMN_OBJECT_NOT_EXIST       = 20003;    // object not exist
const RIVErrCode_t   RIV_COMN_DATA_VERSION_DIFFERENT = 20004;    // different pb has different versions
const RIVErrCode_t   RIV_COMN_NO_DATA_LOADED         = 20005;    // no data loaded

/*
 data is caching or no cache data, this error code returns when getHorizon asks a range not cached.
* if the range is asked to be cached by updatePosition, wait some time until correspodant data cached
* if not, get Horizon will always return RIV_COMN_DATA_CACHEING
*/
// const RIVErrCode_t   RIV_COMN_DATA_CACHING          = 20003; 
}

