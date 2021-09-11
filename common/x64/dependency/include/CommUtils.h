/**
*******************************************************************************
*                       Continental Confidential
*                  Copyright (c) Continental AG. 2018-2019
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
******************************************************************************
* @file   CommUtils.h
* @brief  Declaration of CommUtils
*******************************************************************************
*/
#pragma once

#include <stdint.h>
#include <cmath>

#define DEQUAL(x1, x2) (fabs((x1)-(x2))<0.000001)

//interpolate function
double doLinearInterpolation(double x1, double x2, uint64_t t1, uint64_t t2, uint64_t tn);

//tag info mask
#define SENSOR_TAGGER_TAG_SUCCESS_E  (0x0)
#define SENSOR_TAGGER_TAG_GPS_ERR_E  (0x1)
#define SENSOR_TAGGER_TAG_IMU_ERR_E  (0x2)
#define SENSOR_TAGGER_TAG_OBD_ERR_E  (0x4)
#define SENSOR_TAGGER_TAG_ALL_ERR_E  (0x1 | 0x2 | 0x4)