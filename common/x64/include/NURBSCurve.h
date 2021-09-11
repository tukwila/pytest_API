/*********************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   NURBSCurve.h
 * @brief  The class definition for NURBSCurve.
 *
 * Change Log:
 * Date              Author            Changes
 * 2019-03-18        Tony Xiong        Init version.
 *
 *******************************************************************************
 */

#pragma once

#include <vector>
#include "GeoCurve.h"

namespace YGEO
{
class NURBS;
}

namespace RDBVehicleAPI
{

class NurbsDesc;

class NURBSCurve : public GeoCurve
{
public:   
    /**
     * @brief get the trajectory of a curve, one meter each
    * @return none
    */
   virtual const std::vector<point3D_t>& getTrajectory() override;

   /**
     * @brief get the trajectory of a curve, one meter each
    * @return none
    */
   virtual float64_t getNearestPoint(const point3D_t& point, point3D_t& nearestPoint) const override;

   /**
     * @brief get the remain length along a nurbs curve
     * @INPUT the point to calculate remain length
     * @INPUT is the remain length calculation from start point or end point
    * @return the remain length
    */
   virtual float64_t getRemainLength(const point3D_t& point, bool isFromStartPoint) const override;

   /**
   * @brief the destructor of a nurbs curve
   * @return none
   */
   virtual ~NURBSCurve();

   /**
    * @brief Get NURBS knots.
    * @return A vector containing all NURBS knots.
    */
   const std::vector<float64_t>& getKnots();

   /**
    * @brief Get NURBS control points.
    * @return A vector containing all NURBS control points.
    */
   const std::vector<point3D_t>& getControlPoints();

   /**
    * @brief Get the min value of parameter
    * @return the min value
    */
   float64_t getParamMin() const;

   /**
    * @brief Get the max value of parameter*
    * 1    * @return the max value
    */
   float64_t getParamMax() const;

   const std::vector<std::vector<double>>& getPaintEndPoints() const;  

   double getPaintTotalLength() const;

private:
   NURBSCurve() = delete;
   NURBSCurve(const NURBSCurve& curveObj) = delete;
   NURBSCurve& operator=(const NURBSCurve& curveObj) = delete;
   NURBSCurve(const NurbsDesc& nurbsDes, WGS84_t ref);

   YGEO::NURBS* realNURBS_ = nullptr;

   std::vector<float64_t> trajectoryT_ = {};
   std::vector<uint32_t> trajectoryCurveIndex_ = {};
   std::vector<std::vector<float64_t> > paintEndPoints_ = {};
   std::vector<point3D_t> ctrlPoints_= {};
   std::vector<float64_t> knots_= {};
	float64_t 			paintTotalLength_ = 0; 
   FRIEND_2_ROADDATAINFO;
   friend class GeoLine;
   friend class Curve;
};

} /* namespace geo */
