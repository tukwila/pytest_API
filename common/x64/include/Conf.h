/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Conf.h
 * @brief  Definition of Conf and sun classes
 *******************************************************************************
 */
 
#pragma once
#include <stdint.h>
#include <string>
#include <vector>
#include "VehicleAPICommon.h"
#include "LogicTypes.h"
#include "Geo.h"

// using namespace roaddb::geo;
namespace RDBVehicleAPI
{

struct conf_t
{
    CACHE_MODE_E         cacheMode = CACHE_MODE_WHOLE_E;
    std::string          dbPath = ""; 
    //min GetLayer range searched while getHorizon, it depends on the para range
    layerIndex_t         minGetLayer = 3;
    // WGS84_t              initialPosition;       //used only in realtime mode
    refreshConf_t        refreshConf;           //used only in realtime mode
    threadAttribute_t    cacheThreadAttribute;  //used only in realtime mode
    EXCEPTION_DATA_HANDLE_MODE      dataCheckMode = EDH_MODE_REFUSED;
    EDBType              dbType_ = E_DB_TYPE_PB;
};
}

