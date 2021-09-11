/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   TrafficSign.h
 * @brief  The class definition of TrafficSign.
 *
 * Change Log:
 * Date              Author            Changes
 * 2019-03-18        Tony Xiong        Init version.
 *
 *******************************************************************************
 */

#pragma once

#include <string>

#include "Geo.h"
#include "Object.h"
#include "Foundation.h"

namespace RDBVehicleAPI
{
class Foundation;

class TrafficSign : public Visualization, public std::enable_shared_from_this<TrafficSign>
{
 public:
     ~TrafficSign();

     /**
     * @brief get the TrafficSign type
     * @return the the TrafficSign type
     */
    uint32_t getType() const;

    /**
     * @brief get the position of the TrafficSign
     * @return the postion of the TrafficSign
     */
    WGS84_t getPosition() const;

    int32_t getValue() const;

    std::string getDesc() const;

   
	//SHOW_COUNT_IN_DESTRUCTOR(TrafficSign);

	const std::weak_ptr<const Foundation>& getFoundation() const;

 private:
	void setFoundation(const std::shared_ptr<const Foundation>& pLane);

 private:
    TrafficSign() = delete;

    TrafficSign(const TrafficSign& rhs) = delete;

    TrafficSign& operator=(const TrafficSign& obj) = delete;

    TrafficSign(const objectID_t& id, const uint32_t type, const WGS84_t& position);
 private:
    uint32_t type_ = 0;
    WGS84_t position_;
    int32_t value_ = -1;
    std::string description_ = "";

    std::weak_ptr<const Foundation>	pFoundation_;
    friend class Foundation;
    FRIEND_2_ROADDATAINFO;
};
} /* namespace RDBVehicleAPI */
