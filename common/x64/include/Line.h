/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Curve.h
 * @brief  The class definition for Curve.
 *
 * Change Log:
 * Date              Author            Changes
 * 2019-04-8         Lindun Tang       Init version.
 *
 *******************************************************************************
 */

#pragma once

#include <vector>
#include "VehicleAPICommon.h"
#include "Object.h" 
#include "GeoLine.h"
#include "Curve.h"
#include "Lane.h"

namespace RDBVehicleAPI
{

class Lane;

class Line : public Object
{
 public:
     /**
     * @brief destructor of Line
     */
    ~Line();

    /**
     * @brief Get the Lane which the Line resides
     * @return the Lane object which the Line resides
     */
    const std::vector<std::weak_ptr<const Lane>>& getLanes() const {return pLanes_;}

    /**
     * @brief Get the Line confidence
     * @return the Line confidence
     */
	float64_t getConfidence() const {return confidence_;}
    

    /**
     * @brief Get the Line confidence
     * @return the Line confidence
     */
	const std::shared_ptr<const GeoLine>& getGeoInfo() const {return geoInfo_;}

    /**
     * @brief Get the all the curves in this line.
     * @return all the curves.
     */
    const std::vector<std::shared_ptr<const Curve>>& getCurves() const {return curves_;}

     /**
     * Get Line length
     * @return the length of this line
     */
    float64_t getLength() const;

    /**
     * @brief get the trajectory along a line
     * @INPUT the point to calculate the trajectory
     * @INPUT is the trajectory calculation from start point or end point
     * @INPUT step length of trajectory
     * @OUTPUT the trajectory
     * @return none
     */
    // void getTrajectory(const point3D_t& point, bool isFromStartPoint, uint32_t interval,
    //                      std::vector<point3D_t>& trajectory) const;

    /**
     * @brief Get the predecessor connected lines
     * only support lane boundary, not support evp and centerline
     * @return the predecessor connected lines
     */
    const std::vector<std::weak_ptr<const Line>> &getPreLines();

    /**
     * @brief Get the successor connected lines
     * only support lane boundary,not support evp and centerline
     * @return the  successor connected lines
     */
    const std::vector<std::weak_ptr<const Line>> &getNextLines();

 protected:
    void addLane(std::shared_ptr<const Lane> lanePtr);

 private:
    Line() = delete;
    Line(const Line&) = delete;
    Line& operator=(const Line&) = delete;
    Line(const objectID_t& id, float64_t len, float64_t confidence);
    void addCurve(const std::shared_ptr<const Curve>& pCurve);
    void addCurveLength(const float64_t len);
    bool isLaneLeftBoundary(const std::shared_ptr<const Lane>& lane) const;
    bool isLaneRightBoundary(const std::shared_ptr<const Lane>& lane) const;
    void setConfidence(float64_t confidence) { confidence_ = confidence; }
    
 private:
    std::vector<std::shared_ptr<const Curve>> curves_;
    std::vector<std::weak_ptr<const Line>> vecPredLinePtr_;
    std::vector<std::weak_ptr<const Line>> vecNextLinePtr_;
    float64_t confidence_ = 0.0;
	std::vector<std::weak_ptr<const Lane>>    pLanes_;
    std::shared_ptr<const GeoLine> geoInfo_;

    FRIEND_2_ROADDATAINFO;
    friend class EVP;
};

} /* namespace geo */
