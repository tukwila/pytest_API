/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Foundation.h
 * @brief  The base class for all objects.
 *
 * Change Log:
 * Date              Author            Changes
 * 2019-03-18        Tony Xiong        Init version.
 *
 *******************************************************************************
 */

#pragma once
#include <vector>
#include "Object.h"
#include "TrafficSign.h"

namespace RDBVehicleAPI
{
class TrafficSign;
/**
 * @brief  The class definition for Foundation.
 */
class Foundation : public Object
{
public:
   /**
     * @brief get all traffic sign of this foundation.
     * @return all traffic sign of this foundation.
     */
   const std::vector<std::shared_ptr<const TrafficSign>> &getTrafficSigns() const;

protected:
   /**
     * @brief Construct a new Foundation object
     *
     * @param id the id of this object
     * @param name the id of this object
     */
   Foundation(const objectID_t &id);

private:
   void addTrafficSign(const shared_ptr<const TrafficSign> &pTrafficSign);

private:
   std::vector<std::shared_ptr<const TrafficSign>> vecTrafficsignPtr_;
   FRIEND_2_ROADDATAINFO;
};

} // namespace RDBVehicleAPI
