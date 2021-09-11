/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   DBID2SegID.h
 * @brief  DB ID transfer to segment ID
 *******************************************************************************
 */

#ifndef  DBID_2SEGID_H__
#define  DBID_2SEGID_H__

#include "typeDef.h"

namespace roadDBCore
{

class DBID2SegID
{
public:
    //support node db id, section db id, piece db id
    static SegmentID_t getSegmentID(const uint64_t dbID);
    static bool isDbId(uint64_t ObjId);
    static bool IdValid(int32_t dbId);
};

}
#endif

