/*******************************************************************************
 *                      RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Innertypes.cpp
 * @brief  Vehicle data loader inner types
 *******************************************************************************
 */

#include "VehicleAPICommon.h"

#pragma once

namespace RDBVehicleAPI
{
typedef SP(roadDBCore::VehicleDivisionDetail_t) divisionDataDetailPtr_t;

struct nodeData_t
{
    nodeData_t(const nodeID_t id_)
    :id(id_)
    {
       fromDivisionIds.clear();
       toDivisionIds.clear();
    }

    void addFromDivision(const divisionID_t id)
    {
        fromDivisionIds.insert(id);
    }

    void addToDivision(const divisionID_t id)
    {
        toDivisionIds.insert(id);
    }

    nodeID_t id = 0;
    divisionIDSet_t fromDivisionIds;
    divisionIDSet_t toDivisionIds;
};

struct divisionData_t
{
    divisionData_t(const divisionID_t id_, const nodeID_t fromNodeId_, const nodeID_t toNodeId_)
    :id(id_)
    ,fromNodeId(fromNodeId_)
    ,toNodeId(toNodeId_)
    {

    }
    divisionID_t id = 0;
    nodeID_t fromNodeId = 0;
    nodeID_t toNodeId = 0;
};

enum DATA_LOAD_STATUS_E
{
	DATA_LOAD_STATUS_UNLOAD_E = 0,
	DATA_LOAD_STATUS_OK_E,
	DATA_LOAD_STATUS_FAIL_E

};

typedef SP(void) spDivisionData_t;
typedef std::unordered_map<divisionID_t, spDivisionData_t>  mapDivisionData_t;
typedef std::function<void (const mapDivisionData_t&)> converter_t;
typedef std::function<void (const mapDivisionData_t&)> vehicleDataCallBack_t;
typedef std::unordered_map<segmentID_t, divisionIDSet_t>    seg2DivisionSetMap_t;
// typedef std::unordered_map<divisionID_t, segmentID_t>        division2SegIdMap_t;
typedef std::unordered_map<nodeID_t, SP(nodeData_t)>             mapNodeIds_t;
typedef std::unordered_map<divisionID_t, SP(divisionData_t)>     mapDivisionIds_t;

/* struct division_operations - A collection of methods that manipulate 
 *                              division data and are defined by the user.
 * @converter:Convert the division data format.
 */
struct division_operations_t{
    converter_t converter = nullptr;
};

struct vehicle_conf_t
{
    std::string          dbPath = "";  
    //min GetLayer range searched while getHorizon, it depends on the para range
    layerIndex_t         minGetLayer = 3;
    WGS84_t              initialPosition;       //used only in realtime mode
    refreshConf_t        refreshConf;           //used only in realtime mode
    threadAttribute_t    cacheThreadAttribute;    
    CACHE_MODE_E cacheMode = CACHE_MODE_WHOLE_E; /*data load mode, whole mode or realtime mode*/
    division_operations_t  ops;                 /*call back function when data loaded, used only in realtime mode*/
};
}