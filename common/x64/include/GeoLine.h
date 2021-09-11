/*****************************************************************************
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
 * 2019-12-20        Lindun Tang       Init version.
 *
 *******************************************************************************
 */

#pragma once

#include <vector>
// #include "common.h"
#include "GeoCurve.h"
// #include "Lane.h"

namespace RDBVehicleAPI
{
// class Lane;
class GeoLine : public Geometry
{
   friend class Line;

public:
   GeoLine(float64_t len) : length_(len) {}

   /**
     * Get Line length
     * @return the length of this line
     */
   float64_t getLength() const { return length_; }

   /**
     * @brief Get the start point of this line.
     * @return The start point of this line.
     */
   point3D_t getStartPoint() const { return startPoint_; }

   /**
     * @brief Get the end point of this line.
     * @return The end point of this line.
     */
   point3D_t getEndPoint() const { return endPoint_; }

   /**
     * @brief Get the all the curves in this line.
     * @return all the curves.
     */
   const std::vector<std::shared_ptr<const GeoCurve>> &getCurves() const { return curves_; }

   /**
     * @brief check whether a given point lies inside the area encompassed by two lines
     * @INPUT the point
     * @OUTPUT the other line
     * @return a bool value to indicate whether the given point lies inside
     */
   bool isPointInArea(const point3D_t &point, const std::shared_ptr<const GeoLine> &otherLine) const;

   /**
     * @brief get the nearest point from a given point to this line
     * @INPUT the point
     * @return the given point
     */
   float64_t getNearestPoint(const point3D_t &point, point3D_t &nearestPoint) const;

   /**
     * @brief get the remain length along a nurbs curve
     * @INPUT the point to calculate remain length
     * @INPUT is the remain length calculation from start point or end point
     * @return the remain length
     */
   float64_t getRemainLength(const point3D_t &point, bool isFromStartPoint) const;

   /**
      * @brief get the trajectory, direction and curvature of a line by given step length
      * @INPUT the step length
      * @OUTPUT the trajectory point list
      * @OUTPUT the direction vector list
      * @OUTPUT the curvature list
      * @return none
      */
   void getTrajectory(float64_t stepLen, point3D_t *inputPoint, std::vector<point3D_t> *trajectory,
                        std::vector<point3D_t> *direction, std::vector<float64_t> *curvature, float64_t *lastLegLength) const;

   /**
     * @brief get point along a line from a given point and a specified distance
     * @INPUT point, the point to calculate the trajectory
     * @INPUT isFromStartPoint, is the end point calculation from start point or end point
     * @INPUT distance, the ditance
     * @OUTPUT endPoint, the end point
     * @return none
     */
   void getPointAlongLine(point3D_t point, bool isFromStartPoint, float64_t distance, point3D_t &endPoint) const;

   /**
     * @brief get the cross point with a given point along the given direction in the range of given length
     * @INPUT point, the point
     * @INPUT pos, the pos to indicate the direction
     * @INPUT length, the range
     * @OUTPUT crossPoint, the cross point
     * @return true if there is an intersectioin, false if not
     */
   bool getCrossPoint(point3D_t point, Pose_t pos, float64_t length, point3D_t& crossPoint) const;

   /**
     * @brief get the curvature at a given point on a line
     * @INPUT point, the given point
     * @return the curvature
     */
   float64_t getCurvatureOnPoint(point3D_t point) const;

   /**
     * @brief get the direction at a given point on a lane
     * @INPUT point, the given point
     * @return the direction vector
     */
   direction_t getDirectionOnPoint(point3D_t point) const;

private:
   GeoLine() = delete;
   GeoLine(const GeoLine &) = delete;
   GeoLine &operator=(const GeoLine &) = delete;
   void addGeoCurve(const std::shared_ptr<const GeoCurve> &pCurve);
   void addLength(const float64_t len);

private:
   std::vector<std::shared_ptr<const GeoCurve>> curves_;
   mutable std::vector<point3D_t> trajectory_;
   point3D_t startPoint_;
   point3D_t endPoint_;
   float64_t length_ = 0.0;

   FRIEND_2_ROADDATAINFO;
};

} // namespace RDBVehicleAPI
