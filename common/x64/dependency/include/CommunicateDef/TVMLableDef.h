/**
 * ******************************************************************************
 *                         RoadDB Confidential
 *                    Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   TVMLabelDef.h
 * @brief  Header of TVM driver view label and TVM IPM lable.
 ********************************************************************************
 */

#ifndef _TVM_LABEL_DEF_H_
#define _TVM_LABEL_DEF_H_

#include "typeDef.h"

// local class header files
namespace roadDBCore
{

const uint32_t TVM_DRIVER_VIEW_LABEL_VERSION = RDB_VERSION_MAIN + 1;

enum TVM_DRIVER_VIEW_LABEL_E : uint8_t
{
    //TVM_DRIVER_VIEW_LABEL_UNLABLE_E          = 0,    // UNLABEL
    //TVM_DRIVER_VIEW_LABEL_STATIC_E           = 1,    // STATIC
    TVM_DRIVER_VIEW_LABEL_ROAD_E             = 0,    // ROAD
    TVM_DRIVER_VIEW_LABEL_SIDEWALK_E         = 1,    // SIDEWALK
    TVM_DRIVER_VIEW_LABEL_MARKING_E          = 2,    // MARKING
    TVM_DRIVER_VIEW_LABEL_BUILDING_E         = 3,    // BUILDING
    TVM_DRIVER_VIEW_LABEL_CONCRETE_BARRIER_E = 4,    // CONCRETE BARRIER
    TVM_DRIVER_VIEW_LABEL_FENCE_E            = 5,    // FENCE
    TVM_DRIVER_VIEW_LABEL_CURB_E             = 6,    // CURB
    TVM_DRIVER_VIEW_LABEL_UNDERPASS_E        = 7,    // UNDERPASS

    TVM_DRIVER_VIEW_LABEL_POLE_E             = 8,    // POLE
    TVM_DRIVER_VIEW_LABEL_FRAME_E            = 9,    // FRAME
    TVM_DRIVER_VIEW_LABEL_TRAFFIC_LIGHT_E    = 10,   // TRAFFIC LIGHT
    TVM_DRIVER_VIEW_LABEL_SIGN_E             = 11,   // SIGN
    TVM_DRIVER_VIEW_LABEL_VEGETATION_E       = 12,   // VEGETATION
    TVM_DRIVER_VIEW_LABEL_TERRAIN_E          = 13,   // TERRAIN
    TVM_DRIVER_VIEW_LABEL_SKY_E              = 14,   // SKY
    //TVM_DRIVER_VIEW_LABEL_CREATURE_E         = 17,   // PERSON
    //TVM_DRIVER_VIEW_LABEL_RIDER_E            = 18,   // RIDER
    TVM_DRIVER_VIEW_LABEL_CAR_E              = 15,   // CAR

    TVM_DRIVER_VIEW_LABEL_TRUCK_E            = 16,   // TRUCK
    TVM_DRIVER_VIEW_LABEL_EGO_VEHICLE_E      = 17,   // EGO VEHICLE
    TVM_DRIVER_VIEW_LABEL_MOVABLE_OBSTACLE_E = 18,   // MOVABLE OBSTACLE
    TVM_DRIVER_VIEW_LABEL_NONE_E             = 19,   // NONE FOR PLACEHOLDER

    TVM_DRIVER_VIEW_LABEL_MAX_E
};

enum TVM_IPM_LABEL_E : uint8_t
{
    //TVM_IPM_LABEL_UNLABLE_E                 = 0,    // UNLABEL
    TVM_IPM_LABEL_ROAD_E                    = 0,    // ROAD
    //TVM_IPM_LABEL_OFF_ROAD_E                = 2,    // OFF ROAD
    TVM_IPM_LABEL_SOLID_E                   = 1,    // SOLID
    TVM_IPM_LABEL_DASHED_E                  = 2,    // DASHED
    TVM_IPM_LABEL_ROAD_MARKING_E            = 3,    // ROAD MARKING
    TVM_IPM_LABEL_DASHED_DASHED_DASHED_E    = 4,    // DASHED-DASHED-DAHED
    TVM_IPM_LABEL_DASHED_SOLID_E            = 5,    // DASHED-SOLID

    TVM_IPM_LABEL_SOLID_DASHED_E            = 6,   // SOLID-DASHED
    TVM_IPM_LABEL_DASHED_SOLID_DASHED_E     = 7,   // DASHED-SOLID-DASHED
    TVM_IPM_LABEL_DOUBLE_SOLID_E            = 8,   // DOUBLE SOLID
    TVM_IPM_LABEL_DIRECTION_E               = 9,   // DIRECTION
    TVM_IPM_LABEL_DIVERSION_E               = 10,   // DIVERSION
    TVM_IPM_LABEL_DASHED_DASHED_E           = 11,   // DASHED-DASHED
    //TVM_IPM_LABEL_VEHICLE_E                 = 16,   // VEHICLE
    TVM_IPM_LABEL_CROSSWALK_E               = 12,   // CROSSWALK
    TVM_IPM_LABEL_STOP_LINE_E               = 13,   // STOP LINE
    TVM_IPM_LABEL_DASH_CONNECT_E            = 14,   // DASH CONNECT

    TVM_IPM_LABEL_MAX_E
};

const char *const TVM_DRIVER_VIEW_LABEL_NAME[TVM_DRIVER_VIEW_LABEL_MAX_E] =
{
    "road",
    "sidewalk",
    "marking",
    "building",
    "concreteBarrier",
    "fence",
    "curb",
    "underpass",
    "pole",
    "frame",
    "trafficLight",
    "sign",
    "vegetation",
    "terrain",
    "sky",
    "car",
    "truck",
    "egoVehicle",
    "movableObstacle",
    "none"
};

// semantic point / voxel color
const uint8_t TVM_PT_COLORS[TVM_DRIVER_VIEW_LABEL_MAX_E][3] =
{
    {255, 245, 238}, // road, sea shell
    {  0,   0, 160}, // sidewalk, Navy
    { 64, 255, 255}, // marking, paint,  cyan
    {128,   0,   0}, // building,  dark red
    {255, 182, 193}, // concrete barrier / jersey wall, Light Pink
    { 64,  64, 128}, // fence / guard rail,  midnight Blue
    {173, 255,  47}, // curb,   green yellow
    {  0, 128, 128}, // underpass : bridge / tunnel, Teal
    {192, 192, 128}, // pole,  dark khaki
    { 70, 130, 180}, // frame, steel blue
    {255,  64,   0}, // traffic light,  orange red
    {224,   0, 224}, // sign, magenta
    {0  , 192,   0}, // vegetation, green
    {139,  69,  19}, // terrain, brown
    {255, 255,   0}, // sky, pure yellow
    {139,   0, 139}, // car, dark magenta
    { 70,   0, 130}, // truck, indigo
    {  0,   0, 255}, // ego-vehicle, pure blue
    {218, 165,  32}, // movable obstacles, goldenrod
    {255,   0,   0}, // none, pure red
};


} // namespace roadDBCore

#endif


