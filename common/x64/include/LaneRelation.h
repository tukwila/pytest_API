/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   LaneRelation.h
 * @brief  LaneRelation
 *******************************************************************************
 */

#pragma once

#include "InnerStructs.h"

namespace RDBVehicleAPI
{
class LaneRelation
{
public:
    void clear();
    void add(const objectID_t& srcId, const objectID_t& destId, const LANE_CONNECT_DIR_E drType);
    void addChange(const objectID_t& srcId, const objectID_t& destId);
    void sub(const objectIDSet_t& ids);
    void merge(const LaneRelation& other);
    void conv(const objectIDSet_t& ids, std::unordered_map<objectID_t, SP(LaneConnection)> & outLaneConnects);
private:
    void convItem(const std::unordered_multimap<objectID_t, objectID_t>& src, objectID_t key,objectIDSeq_t& dest);
private:
    std::unordered_multimap<objectID_t, objectID_t>    froms_;
    std::unordered_multimap<objectID_t, objectID_t>    tos_;
    std::unordered_multimap<objectID_t, objectID_t>    lefts_;
    std::unordered_multimap<objectID_t, objectID_t>    rights_;
    std::unordered_multimap<objectID_t, objectID_t>    leftChanges_;
    std::unordered_multimap<objectID_t, objectID_t>    rightChanges_;
};


// }
}
