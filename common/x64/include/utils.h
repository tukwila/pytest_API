/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   utils.h
 * @brief  utils 
 *******************************************************************************
 */

#pragma once

#include <iostream>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <math.h>
#include <pthread.h>
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <boost/function.hpp>
#include "utilsSys.h"
#include "VehicleAPICommon.h"

// using namespace roaddb::geo;
namespace RDBVehicleAPI
{
bool isGpsValid(const WGS84_t& gps);
bool str2WGS84(const std::string& sSrc, WGS84_t& point);
bool str2point3D(const std::string& sSrc, point3D_t& point);
bool getSegsByGps(const WGS84_t& point, segmentIDSeq_t& segs);
bool getSegAnchorsPoint(const  segmentID_t & segId, WGS84_t& anchorsPoint);
bool getSegmentByGps(const WGS84_t& point, segmentID_t& curSeg);
bool getAroundSegments(segmentID_t oriSegID, segmentIDSeq_t& vAroundSegIDs, bool bNeedOri = true);
void convGps2RelativePos(const WGS84_t& pSrc, const WGS84_t& pBasic, point3D_t& pDest);
void convRelativePos2Gps(const point3D_t& pSrc, const WGS84_t& pBasic, WGS84_t& pDest);    
bool getSegOfLayersByGps(const WGS84_t& point, const std::size_t nLayers, segmentID_t& curSegId,segmentIDSeq_t& segs);
void getSegOfLayersBySegmentId(const segmentID_t& curSegId, const std::size_t nLayers, segmentIDSeq_t& segs);
uint getMinGetLayer(const float64_t range);
bool checkGps(const roadDBCore::Point3d_t &gps);
}

