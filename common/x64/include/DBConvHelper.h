/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   db_conv_helper.h
 * @brief  db_conv_helper  
 *******************************************************************************
 */

#pragma once

#include "InnerStructs.h"
#include "DBConnect.h"
#include "jsoncpp.hpp"
//#include "NURBS/NURBS.h"


namespace RDBVehicleAPI
{
void convJson2StdVec(const Json::Value& array, std::vector<std::string>& elems);

//roadDBCore::Point3f_t Conv_RIV2CorePoint(const RIVAPI::point3D_t& src);

//RIVAPI::point3D_t& Conv_Core2RIVPoint(const roadDBCore::Point3f_t& src, RIVAPI::point3D_t& dest);

bool convFormatMarketType(const int32_t type, MARKERTYPE_E& markerType);

bool convFormatNURBS(const std::string& sNurbs, NurbsDesc& nurbs);

bool convRow2Curve(const dbRow_t& row, objectID_t& id, objectID_t& lineId,
    lineIndex_t& lineIndex, uint32_t& type, float64_t& length,
    NurbsDesc& params);



// }
}