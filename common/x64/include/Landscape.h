/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Landscape.h
 * @brief  The class definition of Landscape.
 *
 * Change Log:
 * Date              Author            Changes
 * 2019-03-18        Tony Xiong        Init version.
 *
 *******************************************************************************
 */

#pragma once
#include <vector>
#include <string>
#include "VehicleAPICommon.h"
#include "Line.h"
#include "Junction.h"
#include "Node.h"
#include "RoadEdge.h"

#include <memory>

namespace RDBVehicleAPI
{
class Lane;
class TrafficSign;
class NDACache;
class Line;
class Node;
class RoadEdge;

class Landscape : public Foundation, public std::enable_shared_from_this<Landscape>
{
public:
   /**
     * @brief Destroy the Landscape object
     *
     */
    ~Landscape();

   /**
     * @brief Get all lanes contained in the landscape.
     * @return A list containing all lanes contained in the landscape
     */
   const std::vector<std::shared_ptr<const Lane>> &getLanes() const;

    /**
     * @brief Get pre Landscapes of this Landscape 
     * no matter previous object is landscape or junction
     * @return  Landscapes(pointer).
     */
    const std::vector<std::weak_ptr<const Landscape>>& getPreLandscapes() const;

    /**
     * @brief Get next Landscapes of this Landscape 
     * no matter next object is landscape or junction
     * @return Landscapes(pointer).
     */
    const std::vector<std::weak_ptr<const Landscape>>& getNextLandscapes() const;
    
    /**
     * @brief Get previous Junction of this Landscape
     * @return A Junction(pointer).
     */

    std::weak_ptr<const Junction> getPreJunction() const;

   /**
     * @brief Get next Junction of this Landscape
     * @return A Junction(pointer).
     */
   std::weak_ptr<const Junction> getNextJunction() const;

   /**
     * @brief get the junction which the landscape resides
     * @return the junction which the landscape resides,
     *         null if there is no junction
     */
   const std::weak_ptr<const Junction> getJunction() const;

   /**
     * @brief get the left Edges of the landscape
     * @return the left Edges of the landscape
     */
   const std::vector<std::shared_ptr<const RoadEdge>> &getLeftEdges() const;

   /**
     * @brief get the right Edges of the landscape
     * @return the right Edges of the landscape
     */
   const std::vector<std::shared_ptr<const RoadEdge>> &getRightEdges() const;

   //SPVec(IBarrier) getLeftBarriers() const override;
   //SPVec(IBarrier) getRightBarriers() const override;
   /**
     * @brief get the start node of the landscape
     * @return the  start node of the landscape
     */
   const std::shared_ptr<const Node> getStartNode() const;

   /**
     * @brief get the end node of the landscape
     * @return the  end node of the landscape
     */
   const std::shared_ptr<const Node> getEndNode() const;

   /**
     * @brief get the divisionIDS which generate the landscape
     * @return the divisionIDS which generate the landscape
     */
   const std::set<divisionID_t> &getDivisionIDs() const;

   /**
     * @brief get if the lanes in road are ordered
     * @return the if the lanes in road are ordered
     */
   bool isLanesOrdered() const;


private:
   void setLanesOrdered(const bool bLanesOrdered);
   void setLeftRoadEdges(const std::vector<std::shared_ptr<const RoadEdge>> &pEdges);
   void setRightRoadEdges(const std::vector<std::shared_ptr<const RoadEdge>> &pEdges);
   void addLane(const std::shared_ptr<const Lane> &pLane);
   void addPreLandscape(const std::shared_ptr<const Landscape> &pFromRoad);
   void addNextLandscape(const std::shared_ptr<const Landscape> &pToRoad);
   void setJunction(const std::shared_ptr<const Junction> &pJunction);
   void setStartNode(const std::shared_ptr<const Node> &pFromNode);
   void setEndNode(const std::shared_ptr<const Node> &pToNode);
   void setPreJunction(const std::shared_ptr<const Junction> &preJunction);
   void setNextJunction(const std::shared_ptr<const Junction> &nextJunction);

private:
   //Landscape(const std::shared_ptr<const RoadInVehicleCommonAPI::Landscape>& data);
   Landscape() = default;
   Landscape(const Landscape &rhs) = delete;

   Landscape &operator=(const Landscape &obj) = delete;

   /**
     * @brief the constructor of a Landscape
     * @INPUT the id
     * @INPUT the divisionIds which the landscape resides
     * @return none
     */
    Landscape(const objectID_t& id, const std::set<divisionID_t>& divisionIds = std::set<divisionID_t>());


 private:
    std::vector<std::shared_ptr<const Lane>> lanes_;
    std::vector<std::shared_ptr<const TrafficSign>> trafficSigns_;
    // std::vector<std::shared_ptr<const Line>> vecLaneBoundaries_;

    std::vector<std::weak_ptr<const Landscape>> preLandscapes_;
    std::vector<std::weak_ptr<const Landscape>> nextLandscapes_;

    // std::weak_ptr<const Landscape> preLandscape_;
    // std::weak_ptr<const Landscape> nextLandscape_;

    std::weak_ptr<const Junction> preJunction_;
    std::weak_ptr<const Junction> nextJunction_;

	  std::set<divisionID_t>          divisionIDs_;

	  std::weak_ptr<const Junction> 		    junction_; // avoid memory leak


	  std::vector<std::shared_ptr<const RoadEdge>>			pLeftRoadEdges_;
	  std::vector<std::shared_ptr<const RoadEdge>>		pRightRoadEdges_;
	 //VSPVec(IBarrier)				pLeftBarriers_;
	 //VSPVec(IBarrier)				pRightBarriers_;
	 //VSPVec(Lane)					pLanes_;
	 //VSPVec(IPaint)					pPaints_;
	  std::shared_ptr<const Node> 				pStartNode_;
	  std::shared_ptr<const Node> 				pEndNode_;
	  bool bLanesOrdered_;
	  std::set<objectID_t>       	    laneIDs_ = {};
    
    friend class Junction;

    FRIEND_2_ROADDATAINFO;
};
} /* namespace RDBVehicleAPI */
