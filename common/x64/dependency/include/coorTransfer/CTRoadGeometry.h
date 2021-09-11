/**
 ************************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 ************************************************************************************
 * @file   CTSlamLoc.h
 * @brief  Definitions of common structures
 ************************************************************************************
 */

#ifndef __CT_ROAD_GEOMETRY_H__
#define __CT_ROAD_GEOMETRY_H__


#include <vector>
#include "typeDef.h"
#include "segment/Segment.h"
#include "CommunicateDef/RdbV2SRoadObject.h"
#include "segment/CoordsTransferAlgo.h"
#include "CTCommon.h"
#include "algoInterface/IRoad.h"


namespace roadDBCore
{

namespace coorTrans
{

uint32_t transRgPayload(const SegmentID_t refSeg, const SegmentID_t relSeg,
                        RoadGeometryPayload_t &rgPayload);

} //namespace coorTrans

} //namespace roadDBCore

#endif

