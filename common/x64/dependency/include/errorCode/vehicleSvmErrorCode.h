/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   vehicleSvmErrorCode.h
 * @brief  vehicle svm error code define.
 *******************************************************************************
 */
#ifndef VEHICLE_SVM_ERROR_CODE_H
#define VEHICLE_SVM_ERROR_CODE_H

#include "typeDef.h"
#include "errorCode/moduleMaskCode.h"

namespace roadDBCore
{
// input and system error
const uint32_t SV_ERR_SYSTEM                = VEHICLE_ALGO_SVM;   // 0X20000000 + 0X0C00000  = 549453824
const uint32_t SV_ERR_SYSTEM_INIT           = SV_ERR_SYSTEM + 0X01;
const uint32_t SV_ERR_SYSTEM_RELEASE        = SV_ERR_SYSTEM + 0X02;
const uint32_t SV_ERR_SYSTEM_R2DSNIPPET     = SV_ERR_SYSTEM + 0X03;
const uint32_t SV_ERR_SYSTEM_D2RSNIPPET     = SV_ERR_SYSTEM + 0X04;
const uint32_t SV_ERR_SYSTEM_INIT_DC        = SV_ERR_SYSTEM + 0X05;
const uint32_t SV_ERR_SYSTEM_RELEASE_DC     = SV_ERR_SYSTEM + 0X06;
const uint32_t SV_ERR_SYSTEM_INIT_EVENT     = SV_ERR_SYSTEM + 0X07;
const uint32_t SV_ERR_SYSTEM_RELEASE_EVENT  = SV_ERR_SYSTEM + 0X08;
const uint32_t SV_ERR_SYSTEM_CONFIG_SV      = SV_ERR_SYSTEM + 0X09;
const uint32_t SV_ERR_SYSTEM_CONFIG_TS      = SV_ERR_SYSTEM + 0X0A;
const uint32_t SV_ERR_SYSTEM_INIT_DL        = SV_ERR_SYSTEM + 0X10;  // 16
const uint32_t SV_ERR_SYSTEM_RUN_DL         = SV_ERR_SYSTEM + 0X11;  // 17
const uint32_t SV_ERR_SYSTEM_GET_IMG        = SV_ERR_SYSTEM + 0X12;  // 18
const uint32_t SV_ERR_SYSTEM_SLAM_POSE      = SV_ERR_SYSTEM + 0X13;  // 19
const uint32_t SV_ERR_SYSTEM_SLAM_POINT     = SV_ERR_SYSTEM + 0X14;  // 20
const uint32_t SV_ERR_SYSTEM_PRINT_SNIPPET  = SV_ERR_SYSTEM + 0X15;  // 21
const uint32_t SV_ERR_SYSTEM_PRINT_QUALITY  = SV_ERR_SYSTEM + 0X16;  // 22


// event level
const uint32_t SV_ERR_EVENT_SPARSE          = SV_ERR_SYSTEM          + 0X0A + 0X0A + 0X0A;  // 30
const uint32_t SV_ERR_EVENT_DENSE           = SV_ERR_EVENT_SPARSE    + 0X0A;   // 40
const uint32_t SV_ERR_EVENT_FTRPOINT        = SV_ERR_EVENT_DENSE     + 0X0A;   // 50
const uint32_t SV_ERR_EVENT_FTRLINE         = SV_ERR_EVENT_FTRPOINT  + 0X0A;   // 60
const uint32_t SV_ERR_EVENT_ROADPLANE       = SV_ERR_EVENT_FTRLINE   + 0X0A;   // 70
const uint32_t SV_ERR_EVENT_ONROAD          = SV_ERR_EVENT_ROADPLANE + 0X0A;   // 80
const uint32_t SV_ERR_EVENT_PAVEEDGE        = SV_ERR_EVENT_ONROAD    + 0X0A;   // 90
const uint32_t SV_ERR_EVENT_FURNITURE       = SV_ERR_EVENT_PAVEEDGE  + 0X0A;   // 100
const uint32_t SV_ERR_EVENT_BARRIER         = SV_ERR_EVENT_FURNITURE + 0X0A;   // 110
const uint32_t SV_ERR_EVENT_ACTTS           = SV_ERR_EVENT_BARRIER   + 0X0A;   // 120
    
// event step (offset value to SV_ERR_EVENT_)
const uint32_t SV_ERR_STEP_INIT             = 0X00;
const uint32_t SV_ERR_STEP_START            = 0X01;
const uint32_t SV_ERR_STEP_SETFRAME         = 0X02;
const uint32_t SV_ERR_STEP_SAVEDATA         = 0X03;
const uint32_t SV_ERR_STEP_LOADDATA         = 0X04;
const uint32_t SV_ERR_STEP_ASSEMBLE2D       = 0X05;
const uint32_t SV_ERR_STEP_ASSEMBLE3D       = 0X06;
const uint32_t SV_ERR_STEP_GENMODEL         = 0X07;
const uint32_t SV_ERR_STEP_GETQUALITY       = 0X08;
const uint32_t SV_ERR_STEP_STOP             = 0X09;

}

#endif

