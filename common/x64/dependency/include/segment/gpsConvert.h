/*
*******************************************************************************
*                       Continental Confidential
*                  Copyright (c) Continental AG. 2016-2017
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file   CoordsTransferAlgo.h
* @brief  declaration of gps coordinates transfer functions
*******************************************************************************
*/

#ifndef _GPS_CONVERT_H_
#define _GPS_CONVERT_H_
#include "CommunicateDef/RdbV2SGeometry.h"


namespace roadDBCore
{

/**
 *******************************************************************************
 * @brief calcRelativeLocation - calculate relative location according to standard location
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - Point3d_t &standPoint
 *                 standard gps point.
 *
 *  @param [In]  - Point3d_t &changePoint
 *                 the point which needed calculate relative location
 *
 *  @param [Out]  - Point3_t<T> &outPoint
 *                 relative location based on standard point.
 *
 *  @return status -
 *
 *  <2> Detailed Description:
 *
 *  \ingroup
 *******************************************************************************
 */
template <typename T>
void calcRelativeLocation(const Point3d_t &standPoint,
                          const Point3d_t &changePoint,
                          Point3_t<T> &outPoint);
/**
 *******************************************************************************
 * @brief calcRelativeLocationVec - calculate relative location according to standard location
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - Point3d_t &standPoint
 *                 standard gps point.
 *
 *  @param [In]  - vector<Point3d_t> &changePoint
 *                 vector of points which needed to calculate relative locations
 *
 *  @param [Out]  - vector<Point3_t<T>> &outPoint
 *                 vector of relative locations based on standard point.
 *
 *  @return status -
 *
 *  <2> Detailed Description:
 *
 *  \ingroup
 *******************************************************************************
 */
template <typename T>
void calcRelativeLocationVec(const Point3d_t &standPoint,
                             const std::vector<Point3d_t> &changePoint,
                             std::vector<Point3_t<T>> &outRefPoint);

/**
 *******************************************************************************
 * @brief calcGpsFromRelLocation - calculate gps from relative location and stand gps
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - Point3d_t &standPoint
 *                 standard gps point
 *
 *  @param [In]  - Point3_t<T> &relPoint
 *                 relative position
 *
 *  @param [Out]  - Point3d &outGpsPoint
 *                 out gps point
 *
 *  @return status -
 *
 *  <2> Detailed Description:
 *
 *  \ingroup
 *******************************************************************************
 */
template <typename T>
void calcGpsFromRelLocation(const Point3d_t &standPoint,
                            const Point3_t<T> &relPoint,
                            Point3d_t &outGpsPoint);

/**
 *******************************************************************************
 * @brief calcGpsFromRelLocationVec -
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - Point3d_t &standPoint
 *           standard gps point
 *
 *  @param [In]  - std::vector<Point3_t<T>> &relPoint
 *           vector of relative positions
 *
 *  @param [Out] - vector<Point3d_t> &outGpsPoint
 *                 vector of gps points
 *
 *  @return
 *
 *******************************************************************************
 */

template <typename T, typename P, typename U>
void calcGpsFromRelLocationVec(const Point3_t<T> &standPoint,
                               const std::vector<Point3_t<P>> &refPoint,
                               std::vector<Point3_t<U>> &outGpsPoint);


/**
 *******************************************************************************
 * @brief getGpsDistance - Calculate the distance of two points using gps coords
 *
 *  <1> Parameter Description:
 *
 *  @param [In] - gpsA  one point from the pair
 *
 *  @param [In] - gpsB  another point
 *
 *  @return Distance between the given points
 *
 *  <2> Detailed Description:
 *     This function is applied to distances less than serveral hundreds kilometers
 *
 *******************************************************************************
 */
double getGpsDistance(const Point3d_t& gpsA, const Point3d_t& gpsB);
}


#endif
