/**
 ************************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 ************************************************************************************
 * @file   CTBackendDB.h
 * @brief  Definitions of common structures
 ************************************************************************************
 */

#ifndef __CT_BACKEND_DB_H__
#define __CT_BACKEND_DB_H__


#include <vector>
#include <map>
#include "typeDef.h"
#include "segment/Segment.h"
#include "algoInterface/IServerTransfer.h"
#include "segment/CoordsTransferAlgo.h"
#include "errorCode/serverSysErrorCode.h"


namespace roadDBCore
{

namespace coorTrans
{

uint32_t transDivision2RefSeg(SegmentID_t refSegID, std::vector<std::shared_ptr<DBDivision_t>> &divisions);

uint32_t transDivision2RefSeg(SegmentID_t refSegID, DBDivision_t &division);

uint32_t transReferences2RefSeg(SegmentID_t refSegID,
            std::map<uint64_t, std::vector<std::shared_ptr<DBServerReference_t>>> &references);

uint32_t transNodes2RefSeg(SegmentID_t refSegID, std::map<uint64_t, std::shared_ptr<DBNode_t>> &mapNodes );

uint32_t calSegOfNode(SegmentID_t refSegID, const DBNode_t &stDBNode, SegmentID_t &nodeSegID);

uint32_t isDBDivisionValid(const DBDivisionDetail_t &stDBDivisionDetail, SegmentID_t nodeASegID);

//transfer DBNode_t to it's origin segment
uint32_t transNode2OriSeg(SegmentID_t refSegID, DBNode_t &stDBNode);

//transfer DBNode_t to it's reference segment
uint32_t transNode2RefSeg(SegmentID_t refSegID, DBNode_t &stDBNode );

//transfer DBServerReference
uint32_t transServerReference(const GpsOffsetTransfer& offsetTrans, std::vector<std::shared_ptr<DBServerReference_t>> &vecReferences);

//transfer DBDivision_t to absGps
uint32_t transDivision2AbsGps(std::shared_ptr<DBDivision_t> &spDivision);

//transfer DBNode_t to absGps
uint32_t transNode2AbsGps(std::shared_ptr<DBNode_t> &spNode);

//transfer DBServerReference_t to absGps
uint32_t transReference2AbsGps(std::shared_ptr<DBServerReference_t> &spReference);

} //namespace coorTrans

} //namespace roadDBCore




#endif

