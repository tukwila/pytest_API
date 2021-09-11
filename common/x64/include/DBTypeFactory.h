/**
 *******************************************************************************
 *                         RoadDB Confidential
 *                    Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file  
 * @brief  
 *******************************************************************************
 */

#pragma once

#include <boost/shared_ptr.hpp>

//#include "BaseRoadDataInfo.h"
#include "VehicleAPICommon.h"
#include "LogicTypes.h"

namespace RDBVehicleAPI
{
class BaseRoadDataInfo;

class DBTypeFactory
{
public:
    static std::shared_ptr<RDBVehicleAPI::BaseRoadDataInfo> createRoadDataInfo(EDBType dbType = E_DB_TYPE_PB, EXCEPTION_DATA_HANDLE_MODE checkMode = EDH_MODE_REFUSED);

private:
    DBTypeFactory() = delete;
};

}