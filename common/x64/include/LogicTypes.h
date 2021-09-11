/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   LogicTypes.h
 * @brief  types definition for logic db 
 *******************************************************************************
 */

#pragma once
#include <set>
#include <map>
#include <vector>
#include <string>
#include <stdint.h>

namespace RDBVehicleAPI
{
#define OUT
#define IN

	
typedef double						float64_t;
typedef std::string 				objectID_t;
typedef int32_t				        segmentID_t;
typedef uint64_t                    divisionID_t;


typedef std::vector<segmentID_t>	 segmentIDSeq_t;

typedef std::vector<objectID_t>		 objectIDSeq_t;
typedef std::set<objectID_t>		 objectIDSet_t;

typedef enum
{
    E_DB_TYPE_SQLITE = 0,
    E_DB_TYPE_PB = 1
} EDBType;

 typedef enum
 {
    E_VISUALIZATIONTYPE_UNLABELED = 0,
    E_VISUALIZATIONTYPE_STATIC = 1,
    E_VISUALIZATIONTYPE_ROAD = 2,
    E_VISUALIZATIONTYPE_SIDEWALK = 3,
    E_VISUALIZATIONTYPE_PAINT = 4,
    E_VISUALIZATIONTYPE_BUILDING = 5,
    E_VISUALIZATIONTYPE_WALL = 6,
    E_VISUALIZATIONTYPE_FENCE = 7,
    E_VISUALIZATIONTYPE_CURB = 8,
    E_VISUALIZATIONTYPE_BRIDGE = 9,
    E_VISUALIZATIONTYPE_TUNNEL = 10,
    E_VISUALIZATIONTYPE_POLE = 11,
    E_VISUALIZATIONTYPE_TRAFFICLIGHT = 12,
    E_VISUALIZATIONTYPE_TRAFFICSIGN = 13,
    E_VISUALIZATIONTYPE_VEGETATION = 14,
    E_VISUALIZATIONTYPE_TERRAIN = 15,
    E_VISUALIZATIONTYPE_SKY = 16,
    E_VISUALIZATIONTYPE_PERSON = 17,
    E_VISUALIZATIONTYPE_RIDER = 18,
    E_VISUALIZATIONTYPE_CAR = 19,
    E_VISUALIZATIONTYPE_TRUCK = 20,
    E_VISUALIZATIONTYPE_EGOVEHICLE = 21,
    E_VISUALIZATIONTYPE_VIRTUALWALL = 22,
    E_VISUALIZATIONTYPE_ROADMARK = 23,
    E_VISUALIZATIONTYPE_MAX = 24
 } VisualizationType;

 typedef enum
 {
    E_EVPATTRTYPE_LANE_CHANGE = 0,
    E_EVPATTRTYPE_SPEED_LIMIT = 1,
    E_EVPATTRTYPE_TURN_ATTR = 2,
    E_EVPATTRTYPE_TRAFFIC_SIGN = 3
 } EvpAttributeType;

 typedef enum
 {
    E_ROADEDGETYPE_IMPUTED = 0,
    E_ROADEDGETYPE_PAVEMENT_EDGE = 1,
    E_ROADEDGETYPE_CURB_RAIL = 2,
    E_ROADEDGETYPE_GUARD_RAIL = 3,
    E_ROADEDGETYPE_JERSEY_WALL = 4,
    E_ROADEDGETYPE_MAX = 5
 } RoadEdgeType;

typedef enum
 {
    JUNCTIONTYPE_LANE_MERGE = 0,
    JUNCTIONTYPE_LANE_SPLIT = 1,
    JUNCTIONTYPE_ROAD_MERGE = 2,
    JUNCTIONTYPE_ROAD_SPLIT = 3,
    JUNCTIONTYPE_CROSS = 4,
    JUNCTIONTYPE_MAX = 5
 }JunctionType;
}
