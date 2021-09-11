/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   DataApi.h
 * @brief  The class definition of DataApi.
 *
 * Change Log:
 * Date              Author            Changes
 * 2019-12-15      Yajun Zhao        Move here from DataAPI.
 *
 *******************************************************************************
 */

#pragma once
 
namespace RDBVehicleAPI {
static const uint8_t NONE_TYPE = 0;

static const uint8_t GET_LANE_BY_POSITION_RESPONSE_TYPE = 0xF0;
static const uint8_t GET_LANE_BY_ID_RESPONSE_TYPE = 0xF1;
static const uint8_t GET_LINE_BY_ID_RESPONSE_TYPE = 0xF2;
static const uint8_t GET_NURBSCURVE_BY_ID_RESPONSE_TYPE = 0xF3;
static const uint8_t GET_LANDSCAPE_BY_ID_RESPONSE_TYPE = 0xF4;
static const uint8_t RESPONSE_ERROR_TYPE = 0xFF;

static const uint8_t GET_LANE_BY_POSITION_REQUEST_TYPE = 0xF0;
static const uint8_t GET_LANE_BY_ID_REQUEST_TYPE = 0xF1;
static const uint8_t GET_LINE_BY_ID_REQUEST_TYPE = 0xF2;
static const uint8_t GET_NURBSCURVE_BY_ID_REQUEST_TYPE = 0xF3;
static const uint8_t GET_LANDSCAPE_BY_ID_REQUEST_TYPE = 0xF4;

static const int THREAD_POOL_SIZE = 4;
static const int32_t BUFSIZE = 8192 * 2;
static const int RES_QUEUE_MAX_SIZE = 16;
}
