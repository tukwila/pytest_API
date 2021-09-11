/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   CoorTransfer.h
 * @brief  This class is used to do coordinate transform for key frame, map poi-
 *         nt and road geometry objects.
 *******************************************************************************
 */
#ifndef __COOR_TRANSFER_H__
#define __COOR_TRANSFER_H__

#include <memory>
#include "segment/Segment.h"
#include "CommunicateDef/RdbV2SSlam.h"
#include "CommunicateDef/RdbV2SCommon.h"
#include "coorTransfer/CTSlamLoc.h"
#include "coorTransfer/CTCommon.h"
#include "coorTransfer/CTBackendDB.h"
#include "coorTransfer/CTVehicleDB.h"


namespace roadDBCore
{

namespace coorTrans
{

//transfer snippet files
    //transfer payload
    uint32_t transPayload(SegmentID_t refSegID, SegmentID_t relSegID, std::shared_ptr<PayloadBase_t> &pPayload);

//transfer backend db
    //transfer DivisionDetail to it's origin segment
    uint32_t transDivision2OriSeg(SegmentID_t refSegID,
                                  std::vector<DBDivisionDetail_t> &divisionDetails);
    //transfer DivisionDetail to it's reference segment
    uint32_t transDivision2RefSeg(SegmentID_t refSegID,
                                  std::vector<DBDivisionDetail_t> &divisionDetails);
    //transfer DivisionDetail from it's origin segment to absGps
    uint32_t transDivision2AbsGps(std::vector<DBDivisionDetail_t> &divisionDetails);

//transfer vehicle db
    uint32_t transVehicleDivision2OriSeg(SegmentID_t refSegID, VecVehicleDivision_t &spDivisions);
    uint32_t transVehicleDivision2RefSeg(SegmentID_t refSegID, VecVehicleDivision_t &spDivisions);
    uint32_t transVehicleDivision2RefSeg(SegmentID_t refSegID, std::shared_ptr<VehicleDivisionDetail_t> &spDiv);

    uint32_t transGpsPoints2RefSeg(SegmentID_t refSegID, SegmentID_t srcSegID, std::vector<Point3d_t>&  gpsPoints);
    uint32_t transDivision2RefSeg(SegmentID_t refSegID, DBDivision_t &division);
    uint32_t transDivision2OriSeg(SegmentID_t refSegID, DBDivision_t &division);
    uint32_t transGpsPoints2OriSeg(SegmentID_t refSegID, SegmentID_t srcSegID, std::vector<Point3d_t> & gpsPoints);
    uint32_t transGpsPoints2OriSeg(SegmentID_t refSegID, SegmentID_t srcSegID, std::vector<GpsItem_t> &gpsPoints);
} //namespace coorTrans

} //namespace roadDBCore

#endif
