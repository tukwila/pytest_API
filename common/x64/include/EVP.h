/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   EVP.h
 * @brief  The class definition of EVP.
 *
 * Change Log:
 * Date              Author            Changes
 * 2019-03-18        Tony Xiong        Init version.
 *
 *******************************************************************************
 */

#pragma once

#include <vector>
#include <iostream>
#include <memory>
#include "Line.h"
#include "Lane.h"
#include "SpeedLimit.h"
#include "LaneChange.h"

namespace RDBVehicleAPI
{
class EVPAttribute : public Object, public std::enable_shared_from_this<EVPAttribute>
{
public:
   const WGS84_t &getStartPosition() const;
   const WGS84_t &getEndPosition() const;
   uint32_t getType() const;
   const std::string &getValue() const;

private:
   EVPAttribute() = delete;

   EVPAttribute(const EVPAttribute &rhs) = delete;

   EVPAttribute &operator=(const EVPAttribute &obj) = delete;

   EVPAttribute(const objectID_t &id, const WGS84_t &startPosition, const WGS84_t &endPosition,
                const uint32_t atrributeType, const std::string &atrributeValue);

   WGS84_t startPosition_;
   WGS84_t endPosition_;
   uint32_t type_;
   std::string value_;
   static int s_count_;

   FRIEND_2_ROADDATAINFO;
};

class EVP : public Line, public std::enable_shared_from_this<EVP>
{
public:
   /**
     * @brief Get the line object of this EVP.
     * @return The pointer to the line object which is held by this EVP.
     */
   //  std::shared_ptr<const Line> getLine() const;

    ~EVP();
   /**
     * @brief Get previous EVPs (the direct predecessor EVPs) of this EVP.
     * @return A list containing previous EVPs (pointer).
     */
   const std::vector<std::weak_ptr<const EVP>> &getPreEVPs() const;

   /**
     * @brief Get next EVPs (the direct successor EVPs) of this EVP.
     * @return A list containing next EVPs (pointer).
     */
   const std::vector<std::weak_ptr<const EVP>> &getNextEVPs() const;

   /**
     * Get the Trajectory of this EVP.
     * @param point the given point,if point is nullptr get the first point as start point
     * @param isFromStartPoint a bool value to indicate whether the direction is from start point to
     * end point,The default  is true
     * @param interval the stepLength,The default interval is 2 meters
     * @return the vector of point3D_t
     */
  //  const std::vector<point3D_t> getTrajectory(
  //      const std::shared_ptr<const point3D_t> &point = nullptr,
  //      bool isFromStartPoint = true, int interval = 2) const;

   /**
     * Get the slope of lane at the given point.
     * @param INPUT point the given point
     * @return  float64_t the Cuvature value
     */
  //  float64_t getCurvature(const point3D_t &point) const;

   /**
     * @brief Get the minimum curvature of a line
     * @param INPUT the line
     * @param OUTPUT the minimux curvature
     * @return the minimum curvature
     */
  //  float64_t getMinCurvature() const;

   /**
     * @brief Get the maximum curvature of a line
     * @param INPUT the line
     * @return the maximum curvature
     */
  //  float64_t getMaxCurvature() const;

   /**
     * @brief Determine whether the pose and the evp are in the same direction
     * @param pose the pose to determine with
     * @return true the pose and the evp are in the same direction, otherwise false
     */
  //  bool isSameDirection(const point3D_t &position,
  //                       const pose_t &pose) const;

   /**
     * @brief get the attributes of the evp
     * @return the attributes of the evp
     */
   //const std::vector<std::shared_ptr<const EVPAttribute>> &getAttributes() const;

   /**
     * @brief get all speed limits of the lane.
     * @return all speed limits of the lane.
     */
   const std::vector<std::shared_ptr<const SpeedLimit>> &getSpeedLimits() const { return vecSpeedLimitPtr_; }

   /**
     * @brief get lane changes of the lane.
     * @return all lane changes of the lane.
     */
   const std::vector<std::shared_ptr<const LaneChange>> &getLaneChanges() const { return vecLaneChangePtr_; }

private:
   void addPreEVP(const shared_ptr<const EVP> &pEVP);

   void addNextEVP(const shared_ptr<const EVP> &pEVP);

   void addAttribute(const shared_ptr<const EVPAttribute> &pAttribute);

   EVP() = delete;

   EVP(const EVP &rhs) = delete;

   EVP &operator=(const EVP &obj) = delete;

   EVP(const objectID_t &id, const float64_t confidence = 0,
       const float64_t length = 0, const std::vector<std::shared_ptr<const Curve>> &curves = std::vector<std::shared_ptr<const Curve>>());

private:
   std::vector<std::weak_ptr<const EVP>> vecPreEvpPtr_;
   std::vector<std::weak_ptr<const EVP>> vecNextEvpPtr_;

   std::vector<std::shared_ptr<const SpeedLimit>> vecSpeedLimitPtr_;
   std::vector<std::shared_ptr<const LaneChange>> vecLaneChangePtr_;
   WGS84_t referencePoint_;

   friend class Lane;
   FRIEND_2_ROADDATAINFO;
};
} /* namespace RDBVehicleAPI */
