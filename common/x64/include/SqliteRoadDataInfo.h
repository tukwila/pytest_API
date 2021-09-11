/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   SqliteRoadDataInfo.h
 * @brief  SqliteRoadDataInfo 
 *******************************************************************************
 */

#pragma once

#include "BaseRoadDataInfo.h"

namespace RDBVehicleAPI
{
class SqliteRoadDataInfo : public BaseRoadDataInfo
{
public:
    SqliteRoadDataInfo(EXCEPTION_DATA_HANDLE_MODE mode = EDH_MODE_REFUSED) : BaseRoadDataInfo(mode){}

public:
    bool load(segmentID_t id, const SP_CONST(DBDirectory) pDBDirectory) override;
    void destroy() override;
protected:

    virtual EDBType getDataType() const override {return E_DB_TYPE_SQLITE;}
    bool buildLane2Lane();
    bool buildEvpConnect();
    bool buildLaneInfo();
    // bool buildLineInfo();
    bool buildRoadInfo();
};

}
