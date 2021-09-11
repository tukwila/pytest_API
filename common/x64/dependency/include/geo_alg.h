/*******************************************************************************
*                       Continental Confidential
*                  Copyright (c) Continental AG, 2017

*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file   geo_alg.h
* @brief
*       This file declare the interface of sdk api
*
*
* Change Log:
*      Date              Who               What
*      2017.10.01        Lindun Tang       Create
*******************************************************************************/
#ifndef __GEO_MATH_GEOMETRY_H__
#define __GEO_MATH_GEOMETRY_H__

#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <assert.h>

//#define Refitting_Support

/* Error codes. */
typedef int GEO_STATUS;
#define GEO_SUCCESS 0                  /**< Success. */
#define GEO_FAIL -1                    /**< Common error. */
#define GEO_INVALID_PARAMETER -2       /**< Invalid parameter. */
#define GEO_NULL_PARAMETER -3          /**< NULL parameter. */
#define GEO_DIVIDE_ZERO -4             /**< Divide by zero. */
#define GEO_DISTANCE_NOT_EXHAUSTED 101 /**< Request distance not exhausted. */
/**
 * @brief Struct to define a point or vector.
 */
typedef struct _vec3d
{
    double x; /**< x component. */
    double y; /**< y component. */
    double z; /**< z component. */
} vec3d_t;

/**
 * @brief Struct to define the attitude using Euler angle, the unit is rad.
 * @note The rotate sequence is roll, pitch, yaw. The degree is positive if rotate counter
 * clockwise.
 */
typedef struct _attitude
{
    double yaw;   /**< The angle rotate with z axis. */
    double pitch; /**< The angle rotate with x axis. */
    double roll;  /**< The angle rotate with y axis. */
} attitude_t;

/**
 * @brief Struct to define coeficients of cubic function, the equation is f(t) = a + b*t + c*t^2 +
 * d*t^3.
 */
typedef struct _poly3_coef
{
    double a; /**< The a coeficient. */
    double b; /**< The b coeficient. */
    double c; /**< The c coeficient. */
    double d; /**< The d coeficient. */
} poly3_coef_t;

/**
 * @brief Struct to define a cubic curve.
 */
typedef struct _poly3_curve
{
    poly3_coef_t fx; /**< The cubic function coeficient for x. */
    poly3_coef_t fy; /**< The cubic function coeficient for y. */
    poly3_coef_t fz; /**< The cubic function coeficient for z. */
    double tMin;     /**< The minimum value of argument t. */
    double tMax;     /**< The maximum value of argument t. */

    double newOrder0;
    double newOrder1;
    double scale;
    double scale2;
    bool preOrdered = false;
} poly3_curve_t;

inline void reset(vec3d_t &pt)
{
    pt.x = 0;
    pt.y = 0;
    pt.z = 0;
}

#include <vector>

namespace YGEO
{
class NURBS
{
 public:
    NURBS(const std::vector<vec3d_t> &aCtrlPts,
          const std::vector<double> &aKnots,       // construct with control points and knots vector
          double aMin = 0.0f, double aMax = 1.0f); // default param range is [0, 1]
    NURBS(const NURBS &nurbsCurve);                // copy constructor
    NURBS();                                       // default constructor
    NURBS &operator=(const NURBS &nurbsCurve);     // overload operator
    const std::vector<vec3d_t> &GetCtrlPoints() const;    // get the control points
    const std::vector<double> &GetKnots() const;          // get the knots
    void GetParamRange(double &aMin, double &aMax) const; // get the parameter range of a nurbs
                                                          // curve
    vec3d_t GetPoint(double u) const; // given param u, get the corresponding point
    double GetTotalLength() const;
    double GetNearestPoint(const vec3d_t &point, double &t, vec3d_t &foot) const;
    double getLength(double aMin, double aMax) const; // calc nurbs length from 'aMin' to 'aMax'
    bool getParamByLength(double length, double &t, double &realLen) const;

    const std::vector<poly3_curve_t> &GetCurve() const; // get curve converted from nurbs
    const poly3_curve_t &findCurve(double t, int16_t &index) const;

 private:
    void convertToPoly2(); // convert nurbs to serverl poly2 cure

 private:
    std::vector<poly3_curve_t> vecCurve_; // the poly2 curves converted from nurbs
    std::vector<vec3d_t> vecCtrlPoints_;  // the control points of a nurbs
    std::vector<double> vecKnots_;        // the knots vector of a nurbs
    std::vector<double> legLength_;       // store the length of each leg of nurbs curve
    int32_t degree_;                      // the degress of a nurbs, current been set to 2
    double totalLen_;                     // total length of this nurbs
    double uMin_; // the min value of nurbs domain of definition, current set to 0
    double uMax_; // the max value of nubrs domain of definition, current set to 1
};

/**
 * @brief To get the point which lies after a specific distance after the start point on a nurbs
 * curve.
 * @param[IN] curve_vec    is the vector contain several curve.
 * @param[IN] p            is the given start point which may not on the curve.
 * @param[IN] distance     is the distance along the curve.
 * @param[OUT] index       is the index of of nurbs that has the end point
 * @param[OUT] endPoint       is the point been found.
 * @param[OUT] t           is the param of found point.
 * @param[OUT] rlen        is the remaining length to distance. 0 if this function returns
 * GEO_SUCCESS,
 * @return GEO_SUCCESS if everything is ok, otherwise correspoding error code.
 */
GEO_STATUS get_point_along_nurbs_cluster(const std::vector<NURBS> &curve_vec, const vec3d_t &p,
                                         double distance, uint32_t &index, vec3d_t &endPoint,
                                         double &t, double *rlen = nullptr);

/**
 * @brief To get trajectory, direction vector and curvature from start point of a line
 * @param[IN] curve_vec          is the vector contain several curve.
 * @param[IN] stepLen            is the step length
 * @param[OUT] trajectory        is the trajectory
 * @param[OUT] directionVecList  is the direction vector
 * @param[OUT] curvatureList     is the curvature
 * @return GEO_SUCCESS if everything is ok, otherwise correspoding error code.
 */
GEO_STATUS get_trajectory(const std::vector<NURBS> &curveVec, double stepLen, vec3d_t *startPoint,
                          std::vector<vec3d_t> *trajectory = nullptr,
                          std::vector<vec3d_t> *directionVecList = nullptr,
                          std::vector<double> *curvatureList = nullptr,
                          double *lastLegLength = nullptr);

/**
 * @brief To get remaining length form current position along the nurbs curve.
 * @param[IN] point         is measurement reference point.
 * @param[IN] curveVec     is nurbs curve vector
 * @param[OUT] length       is the length from the point to the end along the curve.
 * @return GEO_SUCCESS if everything is ok, otherwise correspoding error code.
 */
GEO_STATUS get_remaining_length_along_nurbs(const vec3d_t &point,
                                            const std::vector<NURBS> &curveVec, double &length);

/**
 * @brief To check if a point is in the area constructed by two nurbs.
 * @param[IN] point         is measurement reference point.
 * @param[IN] nurbsLeft    is nurbs on the left side
 * @param[IN] nurbsRight   is nurbs on the right side
 * @param[IN] is3D         is the test been carried out on 3d pace or xoy plane
 * @param[OUT] isIn        whether the point is in the area
 * @return GEO_SUCCESS if everything is ok, otherwise correspoding error code.
 */
GEO_STATUS is_point_in_nurbs_area(const vec3d_t &point, const NURBS &nurbsLeft,
                                  const NURBS &nurbsRight, const bool &is3D, bool &isIn);

/**
 * @brief To check if a point is in the area constructed by two nurbs cluster on the xoy plane.
 * @param[IN] point         is measurement reference point.
 * @param[IN] nurbsLeft    is nurbs on the left side
 * @param[IN] nurbsRight   is nurbs on the right side
 * @param[OUT] isIn        whether the point is in the area
 * @return GEO_SUCCESS if everything is ok, otherwise correspoding error code.
 */
GEO_STATUS is_point_in_nurbs_cluster_area(const vec3d_t &point, const std::vector<NURBS> &curveLeft,
                                          const std::vector<NURBS> &curveRight, const bool &is3D,
                                          bool &isIn);

/**
 * @brief To get the nearest point from the given point to the nurbs curve cluster
 * @param[IN] curve        is the nurbs curve vector
 * @param[IN] p            is the given point from where to find the foot.
 * @param[OUT] index       is the index of nurbs curve that has the foot point
 * @param[OUT] foot        is the foot point.
 * @param[OUT] t           is the value of parameter t for the foot point.
 * @return void.using namespace YGEO;
 */
void get_nearest_point_to_nurbs_cluster(const std::vector<NURBS> &nurbsCurve, const vec3d_t &p,
                                        uint32_t &index, vec3d_t &foot, double &t);

/**
 * @brief To get the curvature of a point that on the nurbs on near it.
 * @param[IN] nurbsCurve   is the nurbs curve
 * @param[IN] point        is the point
 * @return the curvature
 */
double get_curvature_on_point(const NURBS &nurbsCurve, const vec3d_t &point);

/**
 * @brief To get slopt of a point that on a nurbs curve or near it
 * @param[IN] nurbsCurve        is the nurbs curve
 * @param[IN] point             is the given point
 * @param[IN] distance          is the distance to determine the endpoint
 * @param[IN] slope             is the slope on that point
 * @return true if the input is valid, false if the input is invalid
 */
bool get_slope_alternative(const NURBS &nurbsCurve, const vec3d_t &point, double distance,
                           double &slope);

/**
 * @brief To check if end point is in the range defined by anchor point and distance
 * @param[IN] nurbsCurve        is the nurbs curve
 * @param[IN] anchorPoint       is the start point of the range
 * @param[IN] endPoint          is the end point to check
 * @param[IN] distance          is the distance of the range
 * @return true if the end point is in range, false if the end point is not in range
 */
bool is_point_in_range(const std::vector<NURBS> &nurbsCurve, const vec3d_t &anchorPoint,
                       const vec3d_t &endPoint, double distance);

/**
 * @brief To get the slope of the reference point(given current point, attitude and distance) relate
 * with current point.
 * @param[IN] point        is the starting position.
 * @param[IN] attitude     is the the attitude of starting position.
 * @param[IN] distance     is the distance to the end position.
 * @param[IN] leftNurbs    is the nurbs to define the left curve of the area.
 * @param[IN] rightNurbs   is the nurbs to define the right curve of the area.
 * @param[OUT] slope       is the slope of the resulting position.
 * @return GEO_SUCCESS if everything is ok, otherwise correspoding error code.
 */
GEO_STATUS get_slope_at_nurbs_point(const vec3d_t &point, const attitude_t &attitude,
                                    double distance, const NURBS &leftNurbs,
                                    const NURBS &rightNurbs, double &slope);

/**
 * @brief To get the slope of the reference point(given current point, attitude and distance) relate
 * with current point.
 * @param[IN] point        is the starting position.
 * @param[IN] attitude     is the the attitude of starting position.
 * @param[IN] distance     is the distance to the end position.
 * @param[IN] leftNurbs    is the nurbs to define the left curve of the adjacent area.
 * @param[IN] rightNurbs   is the nurbs to define the right curve of the adjacent area.
 * @param[OUT] slope       is the slope of the resulting position.
 * @return GEO_SUCCESS if everything is ok, otherwise correspoding error code.
 */
GEO_STATUS get_slope_at_nurbs_adjacent(const vec3d_t &point, const attitude_t &attitude,
                                       double distance, const NURBS &leftNurbs,
                                       const NURBS &rightNurbs, double &slope);

/**
 * @brief To get the camber of the line perpendicular to the center line and go through the
 *        reference point(given current point, attitude and distance).
 * @param[IN] point        is the starting position.
 * @param[IN] attitude     is the attitude of starting point.
 * @param[IN] distance     is the distance to the end position.
 * @param[IN] left_curve   is the nurbs to define the left curve of the area.
 * @param[IN] right_curve  is the nurbs to define the right curve of the area.
 * @param[OUT] camber      is the camber of the reference line.
 * @return GEO_SUCCESS if everything is ok, otherwise correspoding error code.
 * @note For camber definition, please refer to https://en.wikipedia.org/wiki/Camber_angle.
 */
GEO_STATUS get_camber_at_nurbs_point(const vec3d_t &point, const attitude_t &attitude,
                                     double distance, const NURBS &leftNurbs,
                                     const NURBS &rightNurbs, double &camber);

/**
 * @brief To get the start point and end point of a nurbs
 * @param[IN] nurbsCurve   is the nurbs to define the left curve
 * @param[OUT] startPoint  is the start point.
 * @param[OUT] endPoint    is the end point.
 * @return GEO_SUCCESS if everything is ok, otherwise correspoding error code.
 */
GEO_STATUS get_nurbs_start_end_point(const NURBS &nurbsCurve, vec3d_t &startPoint,
                                     vec3d_t &endPoint);

/**
 * @brief To get the cross from a given point to a nurbs
 * @param[IN] nurbsCurve   is the nurbs to define the left curve
 * @param[IN] position     is the given point.
 * @param[IN] attitude     is the attitude of the given point.
 * @param[IN] distance     is the distance along the attitude from the given point.
 * @param[OUT] crossPoint  is the cross point been calculated.
 * @return true if there is a intersection, false if there is not intersection.
 */
bool get_nurbs_cross_point(const NURBS &nurbsCurve, const vec3d_t &position,
                           const attitude_t &attitude, double distance, vec3d_t &crossPoint);

/**
 * @brief To get the intersection status between line segment and nurbs cluster
 * @param[IN] p1   one point of the line segment
 * @param[IN] p2   the other point of the line segment
 * @param[IN] curve_vec the nurbs cluster to intersect
 * @param[OUT] cross_point the cross point if there is a intersection
 * @return true if there is a intersection, or false
 */
bool is_line_segment_intersec_with_nurbs_cluster(const vec3d_t &p1, const vec3d_t &p2,
                                                 const std::vector<NURBS> &curve_vec,
                                                 vec3d_t &cross_point, double *length = nullptr);

/**
 * @brief To get the direction vector at a point which is on the nurbs curve
 * @param[IN] nurbsCurve   is the nurbs to define the left curve
 * @param[IN] point  is the point.
 * @param[OUT] the direction respresented by a tangent vector.
 */
vec3d_t get_direction_on_point(const std::vector<NURBS> &nurbsCurve, const vec3d_t &point);

/**
 * @brief To complete two connecting imcomplete nurbs
 * @param[IN] nurbsFrom     is one of the nurbs
 * @param[IN] nurbsTo       is the other nurbs
 * @param[IN] anchorPoint   is the anchor point to split the joint point cloud
 * @param[IN] footPoint     is one the other foot point to construct the split curve
 * @param[OUT] newNurbsTo   is the newly constructed from cruve
 * @param[OUT] newNurbsTo   is the newly constructed to curve
 * @return GEO_SUCCESS if everything is ok, otherwise correspoding error code.
 */
double get_nurbs_total_len(const NURBS &nurbsCurve);

/**
 * @brief To get the angle between current attitude and the nurbs cluster
 * @param[IN] nurbsCluster  is the nurbs cluster
 * @param[IN] anchorPoint   is the given point
 * @param[IN] attitude      is the attitude of the given point
 * @param[OUT] angle        is the angle with respect to current nurbs
 * @return GEO_SUCCESS if everything is ok, otherwise correspoding error code.
 */
GEO_STATUS get_angle_along_attitude(const std::vector<NURBS> &nurbsCluster, vec3d_t anchorPoint,
                                    attitude_t attitude, double &angle);

/**
 * @brief To get the angle between the tangent vector of specific point of nurbs and the true north
 * @param[IN] nurbsCurve  is the nurbs
 * @param[IN] t           is the parameter corresponding to a specific point on this nurbs
 * @param[OUT] angle      is the angle
 * @return GEO_SUCCESS if everything is ok, otherwise correspoding error code.
 */
GEO_STATUS get_true_north_angle_on_point(const NURBS &nurbsCurve, double t, double &angle);

/**
 * @brief To check whether an intersection exists between two areas
 * @param[IN] area1LeftLine   left line of area 1
 * @param[IN] area1RightLine  right line of area 1
 * @param[IN] area2LeftLine   left line of area 2
 * @param[IN] area2RightLine  right line of area 2
 * @return true if intersect, false if else
 */
bool area_intersection(const std::vector<NURBS> &area1LeftLine,
                       const std::vector<NURBS> &area1RightLine,
                       const std::vector<NURBS> &area2LeftLine,
                       const std::vector<NURBS> &area2RightLine);

} // namespace YGEO

#endif
