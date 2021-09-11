/**
 *******************************************************************************
 *                       RoadDB Confidential
 *       Copyright (c) Continental AG. 2016-2018, RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   CTCommon.h
 * @brief  This class is used to do coordinate transform for Common Objects
 *******************************************************************************
 */

#ifndef __CT_COMMON_H__
#define __CT_COMMON_H__


#include <vector>
#include <opencv2/imgproc.hpp>
#include "typeDef.h"
#include "errorCode/serverSysErrorCode.h"
#include "segment/Segment.h"
#include "segment/CoordsTransferAlgo.h"


namespace roadDBCore
{

struct LmObject_t;
struct LmVolume_t;

namespace coorTrans
{

uint32_t getSegOffset(const SegmentID_t &destSegID, const SegmentID_t &srcSegID, Point3d_t &offset);

//bool transCVMat(cv::Mat &pos, const Point3d_t &offset);

//calculate the origin segment id from a position;
template<typename T>
SegmentID_t calOriginSegment(const SegmentID_t refSeg, const Point3_t<T> &pos)
{
    Point3d_t absPosition;
    Segment segment;

    segment.seg2Gps(refSeg, pos, absPosition);

    SegmentID_t segID = segment.getSegID(absPosition);

    if (segID == INVALID_TILE_ID)
    {
        COM_LOG_ERROR << errorCode(CAL_SECTION_SEGID_FAILED)
                      << "Cal segment id of section failed, ref seg=" << refSeg
                      << ", relative pos=" << pos.relLon
                      << "," << pos.relLat
                      << "," << pos.relAlt
                      << ", absolute pos=" << absPosition.relLon
                      << "," << absPosition.relLat
                      << "," << absPosition.relAlt;

        return INVALID_TILE_ID;
    }

    return segID;
}

void transLmVolume(const GpsOffsetTransfer &offsetTrans, std::shared_ptr<LmVolume_t> &spVolume);

void transLmObject(const GpsOffsetTransfer &offsetTrans, LmObject_t &lmObject);

void transLowPrecisionRT(const GpsOffsetTransfer &offsetTrans,
                         float32_t rotation[9], float32_t translation[3]);


}//coorTrans

}//roadDBCore


#endif
