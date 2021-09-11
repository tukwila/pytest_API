/**
 ************************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017-2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 ************************************************************************************
 * @file   CTLogicDB.h
 * @brief  Definitions of common structures
 ************************************************************************************
 */

#ifndef CT_LOGIC_DB_H_
#define CT_LOGIC_DB_H_

#include <vector>
#include <map>
#include "typeDef.h"

#include "algoInterface/IRoad.h"
#include "segment/CoordsTransferAlgo.h"
#include "coorTransfer/CTRoadGeometry.h"

namespace roadDBCore
{

namespace coorTrans
{
    //used when read from DB

class CTLogicDB
{
public:
    static uint32_t transSRoadsToRefSeg(SegmentID_t refSegID,
                            std::vector<std::shared_ptr<SIntersection_t>> &spIntersections,
                            std::vector<std::shared_ptr<SRoad_t>> &spRoads);
    static uint32_t transSRoadsToOriSeg(SegmentID_t refSegID,
                            std::vector<std::shared_ptr<SIntersection_t>> &spIntersections,
                            std::vector<std::shared_ptr<SRoad_t>> &spRoads);

private:
    static uint32_t transSIntersection(SegmentID_t refSegID,
                                       std::shared_ptr<SIntersection_t> &spIntersection,
                                       bool bOriToRef = true);
    static uint32_t transSRoads(SegmentID_t refSegID,
                                std::vector<std::shared_ptr<SRoad_t>> &spRoads,
                                bool bOriToRef = true);
    static uint32_t transSRoad(SegmentID_t refSegID,
                               std::shared_ptr<SRoad_t> &spRoad,
                               bool bOriToRef = true);
    static uint32_t transSRoadNodes(SegmentID_t refSegID,
               std::map<SegmentID_t, std::set<std::shared_ptr<SRoadNode_t>>> &sRoadNodes,
               bool bOriToRef = true);
    static uint32_t transSRoadDiscreteObjects(SegmentID_t refSegID,
                                              std::set<std::shared_ptr<SRoadDiscreteObject_t>> &sObjs,
                                              bool bOriToRef = true);

private:
    static void transSurface(const GpsOffsetTransfer &offsetTrans, SNurbsSurface_t &surface);
    static void transNURBS(const GpsOffsetTransfer &offsetTrans, NURBS_t &nbs);
    static void transSLines(const GpsOffsetTransfer &offsetTrans,
                            std::map<std::string, std::shared_ptr<SLine_t>> &sLines);
    static void transSEvpAttrs(const GpsOffsetTransfer &offsetTrans,
                               std::vector<std::shared_ptr<SEVP_Attribute_t>> &sEvpAttrs);

};

}

}

#endif

