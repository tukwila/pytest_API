/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   IDebugTransfer.h
 * @brief  Head file of class ITransfer which define the interfaces provided by
 *            transfer in server.
 *******************************************************************************
 */
#ifndef IDEBUG_TRANSTER_H
#define IDEBUG_TRANSTER_H
#include "typeDef.h"
#include <map>
#include <set>
#include <vector>

namespace roadDBCore
{

struct AffectedDivision_t
{
    std::string snippetName;
    std::string snippetType;
    std::string payloadType;
    std::set<uint64_t> partAffectedDiv;
    std::set<uint64_t> fullAffectedDiv;
};

struct RtvDivRelation_t
{
    std::string rtvName;
    std::map<std::string, float32_t> desPercent;
};

class IDebugTransfer
{
public:
    virtual uint32_t putAffectedDivision(const std::vector<AffectedDivision_t> &vecAffectedDivInfo) = 0;
    virtual uint32_t putRtvDivRelation(const std::vector<RtvDivRelation_t> &vecRtvDivRelation) = 0;

};


}

#endif
