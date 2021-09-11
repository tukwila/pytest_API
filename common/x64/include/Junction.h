/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Junction.h
 * @brief  The class definition of Junction.
 *
 * Change Log:
 * Date              Author            Changes
 * 2019-03-18        Tony Xiong        Init version.
 *
 *******************************************************************************
 */

#pragma once
#include <vector>
#include <set>
#include <memory>
#include "VehicleAPICommon.h"
#include "Object.h"
#include "Foundation.h"

#include <memory>

namespace RDBVehicleAPI
{
class Landscape;
class Line;
class Line;
class Junction : public Foundation, public std::enable_shared_from_this<Junction>
{
 public:
    /**
     * @brief Destroy the Junction object
     *
     */
    ~Junction();

    /** 
     * @brief Get junction type .
     * @return the junction type.
     */
    JunctionType getType() const;

    /**
     * @brief Get Incoming Landscapes .
     * @return A list containing Incoming Landscape (pointer).
     */
    // const std::vector<std::shared_ptr<const Landscape>>& getIncomingLandscapes() const;

    // /**
    //  * @brief Outgoing Landscape.
    //  * @return A list containing Outgoing Landscape (pointer).
    //  */
    // const std::vector<std::shared_ptr<const Landscape>>& getOutgoingLandscapes() const;


   /**
     * @brief get the landscapes which reside in the junction
     * @return none
     */
	 const std::vector<std::shared_ptr<const Landscape>>& getLandscapes() const;

   /**
     * @brief get the boundaries of the junction, no data now
     * @return none
     */
    const std::vector<std::shared_ptr<const Line>>& getBoundaries() const;

 private:
    
   /**
     * @brief Junction constructer
     * @return none
     */
    Junction(const objectID_t& id, const JunctionType type = JUNCTIONTYPE_MAX);
    
    Junction() = default;
    Junction(const Junction& rhs) = delete;
    Junction& operator=(const Junction& obj) = delete;

	
    void addLandscape(const std::shared_ptr<const Landscape>& pLandscape);
    void addBoundary(const std::shared_ptr<const Line>& pLine);
    void addIncomingLandscapes(const std::vector<std::weak_ptr<const Landscape>>& pIncomingLandscapes);
    void addOutgoingLandscapes(const std::vector<std::weak_ptr<const Landscape>>&pOutgoingLandscapes);
    void calConnectLandscapes();

 private:
    //  const std::shared_ptr<const RoadInVehicleCommonAPI::Junction> data_;
    JunctionType type_ = JUNCTIONTYPE_MAX;
    std::vector<std::shared_ptr<const Landscape>> pIncomingLandscapes_;
    std::vector<std::shared_ptr<const Landscape>> pOutgoingLandscapes_;


	 std::vector<std::shared_ptr<const Landscape>> 		pLandscapes_;
   //  std::vector<std::shared_ptr<const Lane>> lanes_;
    std::vector<std::shared_ptr<const Line>>      pBoundaries_;
	 std::set<objectID_t> 		landscapeIds_;
	 std::set<objectID_t> 		incomingLandscapeIds_;
	 std::set<objectID_t> 		outgoingLandscapeIds_;

    FRIEND_2_ROADDATAINFO;
};
} /* namespace RDBVehicleAPI */
