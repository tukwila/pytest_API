/*********************************************************
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
#include "./Geo.h"
#include "VehicleAPICommon.h"
#include "Object.h"

namespace RDBVehicleAPI
{

class GeoCurve : public Geometry
{
public:
    /**
     * @brief Get length of this curve.
     * @return The length of this curve.
     */
    float64_t getLength() const {return length_;}

    /**
     * @brief Get type of this curve.
     * @return The type of this curve. Now below types are supported:
     * 0:SIMPLE_LINE,1:POLY3,2:POLY3_CTRLPNT3D,3:SPLINE,4:NURBS
     */
    uint32_t getCurveType() const {return curveType_;}

    /**
     * @brief Get reference point of this curve (NDS).
     * The reference point is the NDS point of the origin of coordinates
     * of the local coordinate system used by this curve.
     * @return The NDS point of the origin of coordinates.
     */
    WGS84_t  getReferencePoint() const {return refPoint_;}

    /**
     * @brief Get the start point of this curve.
     * @return The start point of this curve.
     */
    point3D_t getStartPoint() const {return startPoint_;}

    /**
     * @brief Get the end point of this curve.
     * @return The end point of this curve.
     */
    point3D_t getEndPoint() const {return endPoint_;};

    /**
     * @brief get the trajectory of a curve, one meter each
     * @return none
     */
    virtual const std::vector<point3D_t>& getTrajectory() = 0;

    /**
     * @brief get the nearest point to a given geo curve
     * @OUTPUT the nearest point
     * @return the distance between given point and nearest point
     */
    virtual float64_t getNearestPoint(const point3D_t& point, point3D_t& nearestPoint) const = 0;

    /**
     * @brief get the nearest point to a given geo curve
     * @OUTPUT the nearest point
     * @return the distance between given point and nearest point
     */
    virtual float64_t getRemainLength(const point3D_t& point, bool isFromStartPoint) const = 0;

protected:
    GeoCurve() = default;
    GeoCurve(const GeoCurve& curveObj) = delete;
    GeoCurve& operator=(const GeoCurve& curveObj) = delete;

protected:
   WGS84_t   refPoint_{0, 0, 0};
   point3D_t startPoint_{0, 0, 0};
   point3D_t endPoint_{0, 0, 0};
   float64_t length_{0};
   std::vector<point3D_t> trajectory_ = {};
   uint32_t curveType_;
};

} /* namespace roaddb */

