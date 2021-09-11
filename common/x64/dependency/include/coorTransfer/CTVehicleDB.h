/**
 ************************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 ************************************************************************************
 * @file   CTVehicleDB.h
 * @brief  Definitions of common structures
 ************************************************************************************
 */

#ifndef __CT_VEHICLE_DB_H__
#define __CT_VEHICLE_DB_H__


#include <vector>
#include "typeDef.h"
#include "segment/Segment.h"
#include "algoInterface/IVehicleTransfer.h"
#include "segment/CoordsTransferAlgo.h"
#include "errorCode/serverSysErrorCode.h"


namespace roadDBCore
{

namespace coorTrans
{

    uint32_t isDBDivisionValid(const std::shared_ptr<VehicleDivisionDetail_t> &spDivision, const SegmentID_t nodeASegID);

    uint32_t transVehicleNode2OriSeg(SegmentID_t refSegID, std::map<uint64_t, std::shared_ptr<VehicleNode_t>> &spNodes);

    uint32_t transVehicleNode2RefSeg(SegmentID_t refSegID, std::map<uint64_t, std::shared_ptr<VehicleNode_t>> &spNodes);

    uint32_t transVehicleReference(const GpsOffsetTransfer& offsetTrans, std::vector<std::shared_ptr<VehicleReference_t>> &spReferences);



} //namespace coorTrans

} //namespace roadDBCore




#endif

