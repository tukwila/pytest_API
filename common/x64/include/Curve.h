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
 * 2019-12-20        Lindun Tang       Init version.
 *
 *******************************************************************************
 */

#pragma once

#include <vector>

#include "VehicleAPICommon.h"
#include "Object.h"
#include "GeoCurve.h"
#include "NURBSCurve.h"

namespace RDBVehicleAPI
{

class NurbsDesc;
class Line;

class Curve : public Object
{
public:
  /**  
     * @brief Get curve type from common api
     * @return The Semantic curve type.	
        LINETYPE_SOLID = 0;
        LINETYPE_DASHED = 1;
        LINETYPE_IMPUTED = 2;
        LINETYPE_SLAM_TRACE = 3; //center line
        LINETYPE_UNLABELED = 4;
        LINETYPE_DOUBLE_SOLID = 5;
        LINETYPE_DOUBLE_DASHED = 6;
        LINETYPE_DASHED_SOLID = 7;
        LINETYPE_SOLID_DASHED = 8;
        LINETYPE_DASHED_DASHED_DASHED = 9;
        LINETYPE_DASHED_SOLID_DASHED = 10;
        LINETYPE_EVP = 11; //expect vehicle path
        LINETYPE_ROAD_EDGE = 12;
        LINETYPE_MAX = 13;
   */
   uint32_t getType() const;


  /**
    * @brief Get the geo information of this curve
    * @return a shared ptr to the geo information
    */
   const std::shared_ptr<const GeoCurve>& getGeoInfo() const;

  /**
    * @brief get the line that contain this curve
    * @return the line
    */
   std::weak_ptr<const Line> getLine() const;

    /**
    * @brief Get NURBS knots.
    * @return A vector containing all NURBS knots.
    */
   const std::vector<float64_t>& getKnots() const;

   /**
    * @brief Get NURBS control points.
    * @return A vector containing all NURBS control points.
    */
   const std::vector<point3D_t>& getControlPoints() const;

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

    /**
     * @brief Get reference point of this curve (NDS).
     * The reference point is the NDS point of the origin of coordinates
     * of the local coordinate system used by this curve.
     * @return The NDS point of the origin of coordinates.
     */ 
    WGS84_t getReferencePoint() const;

    /**
     * @brief Get endPoints point of this curve if it's nubrs
     * @return The NDS point of the origin of coordinates.
     */ 
    const std::vector<std::vector<double>>& getPaintEndPoints() const;  

    /**
     * @brief Get Length  of this curve 
     * @return The NDS point of the origin of coordinates.
     */ 
    double getLength() const;     //curveLength

   /**
     * @brief Get paintTotalLength  of this curve if it's nubrs
     * @return The NDS point of the origin of coordinates.
     */ 
    double getPaintTotalLength() const;  

   ~Curve();
   
private:
   Curve() = delete;
   Curve(const Curve& curveObj) = delete;
   Curve& operator=(const Curve& curveObj) = delete;

   Curve(const objectID_t& id, const NurbsDesc& curveDes, uint32_t type, WGS84_t ref);

   bool isNURBSCurve(std::shared_ptr<NURBSCurve>& nurbsInfo) const;
   
   void setLine(const std::shared_ptr<const Line>& pLine);
   void setExpressID(const std::string& expressID) {expressID_ = expressID;}
   std::string getExpressID() const {return expressID_;}
   uint32_t type_;
   std::shared_ptr<const GeoCurve> geoInfo_ = nullptr;
   std::weak_ptr<const Line> linePtr_;
   std::string expressID_;
  FRIEND_2_ROADDATAINFO;
  friend class Line;
  friend class GetCurveByIDResponseMsg;
};

} // namespace RDBVehicleAPI
