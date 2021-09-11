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

#ifndef __CT_SLAM_LOC_H__
#define __CT_SLAM_LOC_H__


#include <vector>
#include "typeDef.h"
#include "segment/Segment.h"
#include "CommunicateDef/RdbV2SSlam.h"
#include "CommunicateDef/RdbV2SLandmark.h"
#include "segment/CoordsTransferAlgo.h"
#include "CTCommon.h"


namespace roadDBCore
{

namespace coorTrans
{

void transSlamSnippet(const GpsOffsetTransfer& offsetTrans,
                      SlamSnippetSR_t &stSlamSnippet);

void transSlamPayload(SegmentID_t dstSeg,
                      SegmentID_t srcSeg,
                      SlamSnippetPayload_t& stSlamSnippetPayload);

void transLmSnippet(const GpsOffsetTransfer &offsetTrans, LmSnippetSR_t &lmSnippet);

void transLmPayload(SegmentID_t dstSeg,
                    SegmentID_t srcSeg,
                    LmSnippetPayload_t& stLmSnippetPayload);

} //namespace coorTrans

} //namespace roadDBCore

#endif
