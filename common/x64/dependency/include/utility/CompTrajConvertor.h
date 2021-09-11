/**
 *******************************************************************************
 *                         RoadDB Confidential
 *                    Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   CompTrajConvertor.h
 * @brief  struct of sensor input
 ******************************************************
*/
#pragma once

#include "algoInterface/Localization/LocCompTrajectory.h"
#include "CommunicateDef/RdbV2SSlam.h"

namespace roadDBCore
{

class CompTrajConvertor
{
public:
    CompTrajConvertor(roadDBCore::Point3d_t& refGps, roadDBCore::uint64_t baseTimestamp);

    algo::vehicle::LocCompTrajectory_t operator ()(const roadDBCore::KeyFrameSR_t& keyframe);

private:
    roadDBCore::Point3d_t refGps_;
    roadDBCore::uint64_t baseTimestamp_;
};

}