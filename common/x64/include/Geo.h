/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Geo.h
 * @brief  The class defines some commonly used types, alias, constants etc.
 *
 * Change Log:
 * Date              Author            Changes
 * 2019-03-18        Tony Xiong        Init version.
 *
 *******************************************************************************
 */

#pragma once

#include "VehicleAPICommon.h"

namespace RDBVehicleAPI
{

struct direction_t
{
    direction_t():x(0), y(0), z(0){}
    direction_t(float64_t _x, float64_t _y, float64_t _z):x(_x), y(_y), z(_z){}
    float64_t   x = 0;
    float64_t   y = 0;
    float64_t   z = 0;
};


class Geo
{
public:
	// static const float64_t PI = 3.141592653589793;

};

} /* namespace geo */
