/*********************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   IObject.h
 * @brief  The interface class definition for objects.
 *
 * Change Log:
 * Date              Author            Changes
 * 2019-03-18        Tony Xiong        Init version.
 *
 *******************************************************************************
 */

#pragma once

#include "VehicleAPICommon.h"
#include "VehicleAPICommon.h"

namespace RDBVehicleAPI{
/**
 * @brief  The interface class definition for objects.
 */
class IObject
{
 public:
    virtual objectID_t getID() const = 0;
};
} /* namespace roaddb */
