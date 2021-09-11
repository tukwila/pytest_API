/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Lane.h
 * @brief  The class definition of lane.
 *
 * Change Log:
 * Date              Author            Changes
 * 2019-03-18        Tony Xiong        Init version.
 *
 *******************************************************************************
 */

#pragma once

#include <vector>
#include <utility>
#include <memory>
#include "VehicleAPICommon.h"
#include "Line.h"
#include "Paint.h"
#include "TrafficSign.h"
#include "Landscape.h"
#include "Foundation.h"
#include "LogicTypes.h"
// #include "BaseRoadDataInfo.h"

#include <memory>

namespace RDBVehicleAPI
{
  class Line;
  class EVP;
  class Landscape;
  class Junction;
  class Line;
  class Paint;


  class Lane : public Foundation, public std::enable_shared_from_this<Lane>
  {
  public:
    ~Lane();
    /**
     * @brief Get all paths contained in the lane.
     * @return A list containing all (expected vehicle) paths contained in the lane
     */
    const std::vector<std::shared_ptr<const EVP>> &getEVPs() const;

    /**
     * @brief Get the left lane boundary.
     * @return a weak point to the left boundary
     */
    std::shared_ptr<const Line> getLeftLaneBoundary() const;

    /**
     * @brief Get the right lane boundary.
     * @return a weak point to the right boundary
     */
    std::shared_ptr<const Line> getRightLaneBoundary() const;

    /**
     * @brief Get the center line of this lane.
     * @return A pointer pointing to the center line of this lane.
     */
    std::shared_ptr<const Line> getCenterLine() const;

    /**
     * @brief Get the landscape in which the lane resides.
     * @return The pointer to the landscape in which the lane resides.
     */
    std::weak_ptr<const Landscape> getLandscape() const;

    /**
     * @brief Get the speed limit of this lane.
     * @return the lane's lowest speed limit
     *
     * Notes:
     * - If the speed limit is not available, an error code will be returned.
     * - If there are multiple speed limit changes in this lane, return the lowest speed limit.
     */
    //uint32_t speedLimit() const;

    /**
     * @brief Get the left adjacent lane
     * @return the left adjacent lane
     */
    std::weak_ptr<const Lane> getLeftAdjLane() const;

    /**
     * @brief Get the right adjacent lane
     * @return the right adjacent lane
     */
    std::weak_ptr<const Lane> getRightAdjLane() const;

    /**
     * @brief Get the predecessor lane
     * @return the predecessor lanes
     */
    const std::vector<std::weak_ptr<const Lane>> &getPreLanes() const;

    /**
     * @brief Get the successor lane
     * @return the  successor lanes
     */
    const std::vector<std::weak_ptr<const Lane>> &getNextLanes() const;

    /** 
     * Get paints of left boundary of this lane
     * @return paints of left boundary of this lane
     */
    const std::vector<std::shared_ptr<const Paint>> &getLeftBoundaryPaints() const { return leftBoundaryPaints_; }

    /** 
     * Get paints of right boundary of this lane
     * @return paints of right boundary of this lane
     */
    const std::vector<std::shared_ptr<const Paint>> &getRightBoundaryPaints() const { return rightBoundaryPaints_; }


private:

    void addEVP(const shared_ptr<const EVP> &pEvp);

    void setLandScape(const shared_ptr<const Landscape> &pLandScape);

    void addPreLane(const shared_ptr<const Lane> &pPreLane);

    void addNextLane(const shared_ptr<const Lane> &pNextLane);

    void setLeftLine(const shared_ptr<const Line> &pLine);

    void setRightLine(const shared_ptr<const Line> &pLine);

    void setCenterLine(const shared_ptr<const Line> &pLine);


    void setLeftAdjLane(const std::shared_ptr<const Lane> &leftLane);
    void setRightAdjLane(const std::shared_ptr<const Lane> &rightLane);

    void addLeftBoundaryPaints(const std::shared_ptr<const Paint>& pPaint) 
    {
        leftBoundaryPaints_.push_back(pPaint);
    }

    void addRightBoundaryPaints(const std::shared_ptr<const Paint>& pPaint) 
    {
        rightBoundaryPaints_.push_back(pPaint);
    }

    float64_t getCrossAngle(NDSPoint_t position, Pose_t pose) const;

    Lane(const objectID_t &id);
    Lane() = delete;
    Lane(const Lane &obj) = delete;
    Lane &operator=(const Lane &obj) = delete;

  private:
    std::vector<std::shared_ptr<const EVP>> vecEvpPtr_;
    std::vector<std::shared_ptr<const TrafficSign>> vecTrafficsignPtr_;

    std::vector<std::weak_ptr<const Lane>> vecPredLanePtr_;
    std::vector<std::weak_ptr<const Lane>> vecSuccLanePtr_;

    std::weak_ptr<const Landscape> pLandscape_;
    std::weak_ptr<const Junction> junctionPtr_;

    std::weak_ptr<const Lane> leftAdjLanePtr_;
    std::weak_ptr<const Lane> rightAdjLanePtr_;

    std::shared_ptr<const Line> leftLaneBoundaryPtr_;
    std::shared_ptr<const Line> rightLaneBoundaryPtr_;

    std::vector<std::shared_ptr<const Paint>> leftBoundaryPaints_;
    std::vector<std::shared_ptr<const Paint>> rightBoundaryPaints_;

    std::shared_ptr<const Line> centerLinePtr_;

    //VSPVec(IRoadMark)       	    pRoadMarks_;
    //VSPVec(IPaint)					pPaints_;

    FRIEND_2_ROADDATAINFO;
    friend class Landscape;
    friend class EVP;
  };
} /* namespace RDBVehicleAPI */
