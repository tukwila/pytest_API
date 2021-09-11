/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   HorizonManager.h
 * @brief  The class definition of HorizonManager.
 *
 * Change Log:
 * Date              Author            Changes
 * 2020-07-28        Alvin             Init version
 *
 *******************************************************************************
 */

#pragma once

#include "Horizon.h"
#include "VehicleAPICommon.h"
#include "utility/Singleton.h"

#include <map>

namespace RDBVehicleAPI {

#define DEFAULT_HORIZON_ID 8888

class HorizonManager
{
public:
    /**
     * @brief Build horizon
     * @param position the position of the start point for building horizon
     * @param lane the lane object of the start point for building horizon
     * @param distance the distance of horizon
     * @return the shared_ptr object of newly built horizon
     */
    shared_ptr<Horizon> buildHorizonByManager(const NDSPoint_t &position, std::shared_ptr<const Lane> lane, uint32_t distance);
    shared_ptr<Horizon> update(uint32_t pathID, uint32_t offset);

    shared_ptr<Horizon> getHorizonByID(uint32_t id);

    void clear() { mapHorizon_.clear(); }

 private:
    std::map<int, shared_ptr<Horizon>> mapHorizon_; // Use map for extension.
};

#define SGR_HORIZON_MANAGER roadDBCore::Singleton<HorizonManager>::getInstance()

} /* namespace RDBVehicleAPI */
