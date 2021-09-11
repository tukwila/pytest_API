/**
 *******************************************************************************
 *                         RoadDB Confidential
 *                    Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   LocCompTrajectory.h
 * @brief  struct of sensor input
 ******************************************************
*/
#pragma once

#include "typeDef.h"
#include "CommunicateDef/RdbV2SGeometry.h"

namespace algo{ namespace vehicle{

enum LOC_COMPENSATE_TRAJECTORY_TYPE_E: int32_t
{
    LOC_COMPENSATE_TRAJECTORY_TYPE_INVALID_E = -1,  // Compensate invalid
    LOC_COMPENSATE_TRAJECTORY_TYPE_TIME_E = 0,      // Compensate with time, default
    LOC_COMPENSATE_TRAJECTORY_TYPE_SPACE_E = 1,     // Compensate with space
    LOC_COMPENSATE_TRAJECTORY_TYPE_TIMESPACE_E = 2, // Compensate both with time and space
    LOC_COMPENSATE_TRAJECTORY_TYPE_TIME_WITH_FRAME_SKIP_E = 3,      // Compensate with time with frame skip
    LOC_COMPENSATE_TRAJECTORY_TYPE_MAX_E
};

struct LocCompTrajectory_t
{
    //timestamps
    roadDBCore::int32_t frmIdx = 0;
    roadDBCore::float64_t taggedTs = 0.0; // ms
    roadDBCore::float64_t triggerImgAbsGlbTs = 0.0;
    roadDBCore::float64_t rcvTs = 0.0;
    roadDBCore::float64_t reportAbsGlbTs = 0.0; // ms
    roadDBCore::float64_t reportTaggedTs = 0.0; // ms

    //before
    roadDBCore::Point3d_t absPosBeforeComp;
    roadDBCore::Point3d_t posi3SigmaBefComp; //m
    roadDBCore::algo_float_t deg_3sigma_YawBefComp = 0.0;

    roadDBCore::algo_float_t degYawBeforeComp_NED = 0.0;
    roadDBCore::algo_float_t degPitchBeforeComp_NED = 0.0;
    roadDBCore::algo_float_t degRollBeforeComp_NED = 0.0;

    //after
    bool compenValid = false;
    bool bSkipped = false;
    roadDBCore::float64_t compTimeMS = 0.0;
    roadDBCore::Point3d_t absPosAfterComp;
    roadDBCore::algo_float_t degYawAfterComp_NED = 0.0;
    roadDBCore::algo_float_t sigma_x_VinV = 0.0;
    roadDBCore::algo_float_t sigma_y_VinV = 0.0;

    roadDBCore::algo_float_t speed = 0.0; // m/s

    roadDBCore::algo_float_t cov = 0.0;
};

}}