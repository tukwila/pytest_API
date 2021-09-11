/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Calc.h
 * @brief  This class defines some helper functions for geo-calculation.
 *
 * Change Log:
 * Date              Author            Changes
 * 2019-04-8         Lindun Tang       Init version.
 *
 *******************************************************************************
 */

#pragma once

#include "Line.h"
#include "VehicleAPICommon.h"
#include "Geo.h"

namespace RDBVehicleAPI
{
const double MATH_HALF_PI = 1.570796326795;
const double FLOAT_OP_ERROR_TRESH = 0.000001;
class Calc
{
public:
    /**
     * @brief Get the linear distance between two point
     * @param INPUT one point
     * @param INPUT the other point
     * @return the distance
     */
    static float64_t getTwoPointDistance(const point3D_t &pointA, const point3D_t &pointB);

    /**
     * @brief Get the rotate angle from direction vector fromDirec to direction vector toDirec
     * @param INPUT the from vector
     * @param INPUT the to vector
     * @param OUTPUT the angle
     * @return the error code
     */
    static float64_t getRotateAngle(const direction_t &fromDirec, const direction_t &toDirec);

    /**
     * @brief Get the end point from given point, along given direction and distance
     * @param INPUT the given point
     * @param INPUT the direction
     * @param INPUT the distance
     * @return the end point
     */
    static point3D_t getEndPoint(const point3D_t &point, const direction_t &direct,
                                 const float64_t &distance);

    /**
     * @brief convert wgs84 coordinate to nds coordinate
     * @param INPUT the wgs84 coordinate
     * @return the nds
     */
    static NDSPoint_t WGS2NDS(const WGS84_t &wgs);

    /**
     * @brief convert nds coordinate to wgs84 coordinate
     * @param INPUT the nds coordinate
     * @return the gps
     */
    static WGS84_t NDS2WGS(const NDSPoint_t &nds);

    /**
     * @brief convert nds coordinate to local coordinate
     * @param INPUT the nds coordinate
     * @param OUTPUT the local coordinate
     * @param INPUT the reference point
     * @return the error code
     */
    static point3D_t NDS2Local(const NDSPoint_t &nds, const WGS84_t &referencePoint);

    /**
     * @brief convert local coordinate to nds coordinate
     * @param INPUT the local coordinate
     * @param OUTPUT the nds coordinate
     * @param INPUT the reference point
     * @return the error code
     */
    static NDSPoint_t Local2NDS(const point3D_t &local, const WGS84_t &referencePoint);

    /**
     * @brief convert from attitude to direction vector
     * @param INPUT the attitude
     * @param OUTPUT the direction vector
     * @return the error code
     */
    static direction_t directionConversion(const Pose_t &pos);

    /**
     * @brief get the cross point between two line segment
     * @param INPUT one point of line segment A
     * @param INPUT one point of line segment A
     * @param INPUT one point of line segment B
     * @param INPUT one point of line segment B
     * @param OUTPUT the cross point which lies between point q1 and point q2
     * @return true if there is an intersection, or false if not
     */
    static bool getLineSegmentCrossPoint(point3D_t p1, point3D_t p2, point3D_t q1, point3D_t q2, point3D_t &crossPointQ);
};

} // namespace RDBVehicleAPI
