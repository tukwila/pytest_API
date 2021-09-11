/**
 ************************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 ************************************************************************************
 * @file   RdbV2SGeometry.h
 * @brief  Definitions of basic geometries
 ************************************************************************************
 */
#ifndef RDB_V2S_GEOMETRY_H
#define RDB_V2S_GEOMETRY_H

#include <vector>           // std::vector
#include <memory>           // std::shared_ptr
#include <cstring>          // std::shared_ptr
#include "typeDef.h"        // uint16 etc.
#include "RdbV2SCommon.h"   // ObserveKfInfo_t

namespace roadDBCore
{

enum GEOMETRY_TYPE_E : uint8_t
{
    GEOMETRY_TYPE_POINT_E = 0,
    GEOMETRY_TYPE_LINE_E,
    GEOMETRY_TYPE_SURFACE_E,
    GEOMETRY_TYPE_VOLUME_E,
    GEOMETRY_TYPE_MAX_E
};

struct Geometry_t
{
    GEOMETRY_TYPE_E eGeometryType;

    Geometry_t(): eGeometryType(GEOMETRY_TYPE_E::GEOMETRY_TYPE_POINT_E) {}

    Geometry_t(GEOMETRY_TYPE_E eInGeometryType): eGeometryType(eInGeometryType) {}
};

/***********************************************
 * Points
 ***********************************************/
enum POINT_TYPE_E : uint8_t
{
    POINT_TYPE_2D_E = 0,
    POINT_TYPE_3D_E,
    POINT_TYPE_2F_E,
    POINT_TYPE_3F_E,
    POINT_TYPE_2I_E,
    POINT_TYPE_3I_E,
    POINT_TYPE_2S_E,
    POINT_TYPE_3S_E,
    POINT_TYPE_2B_E,
    POINT_TYPE_3B_E,
    POINT_TYPE_2F_FEATURE_E,
    POINT_TYPE_3F_FEATURE_E,
    POINT_TYPE_MAX_E
};

struct Point_t : public Geometry_t
{
    POINT_TYPE_E ePointType;

    Point_t(POINT_TYPE_E eInPointType): Geometry_t(GEOMETRY_TYPE_POINT_E), ePointType(eInPointType) {}
};

template <typename T>
struct Point3TypeTrais;

template <typename T>
struct Point3_t : public Point_t
{
    T relLon;
    T relLat;
    T relAlt;

    Point3_t(): Point_t(Point3TypeTrais<T>::value), relLon(0), relLat(0), relAlt(0) {}
    Point3_t(T lon, T lat, T alt): Point_t(Point3TypeTrais<T>::value),  relLon(lon), relLat(lat), relAlt(alt) {}

    void addOffset(float32_t x, float32_t y, float32_t z)
    {
        relLon += x;
        relLat += y;
        relAlt += z;
    }

    template <typename T2>
    void addOffset(const Point3_t<T2>& offset)
    {
        relLon += offset.relLon;
        relLat += offset.relLat;
        relAlt += offset.relAlt;
    }

    Point3_t operator +(const Point3_t<T>& p1) const
    {
       Point3_t<T>  result;
       result.relLon = relLon + p1.relLon;
       result.relLat = relLat + p1.relLat;
       result.relAlt = relAlt + p1.relAlt;
       return result;
    }

    Point3_t operator -(const Point3_t<T>& p1) const
    {
       Point3_t<T>  result;
       result.relLon = relLon - p1.relLon;
       result.relLat = relLat - p1.relLat;
       result.relAlt = relAlt - p1.relAlt;
       return result;
    }

    Point3_t operator *(const float32_t& coef) const
    {
       Point3_t<T>  result;
       result.relLon = relLon * coef;
       result.relLat = relLat * coef;
       result.relAlt = relAlt * coef;
       return result;
    }

    std::string toString()  const
    {
       std::stringstream  str;
       str << relLon << ","
           << relLat << ","
           << relAlt;
       return str.str();
    }
};

template <typename T>
struct Point3Feature_t : public Point3_t<T>
{
    std::vector<uint8_t [32]> vecDescriptors;
};

template <typename T>
struct Point2TypeTrais;

template <typename T>
struct Point2_t : public Point_t
{
    T x;
    T y;

    Point2_t(): Point_t(Point2TypeTrais<T>::value), x(0), y(0) {}
    Point2_t(T tx, T ty): Point_t(Point2TypeTrais<T>::value), x(tx), y(ty) {}
};

template <typename T>
struct Point2Feature_t : public Point2_t<T>
{
    std::vector<uint8_t [32]> vecDescriptors;
};

typedef Point3_t<float64_t> Point3d_t;
typedef Point3_t<float32_t> Point3f_t;
typedef Point3_t<int32_t>   Point3i_t;
typedef Point3_t<int16_t>   Point3s_t;
typedef Point3_t<int8_t>    Point3b_t;
typedef Point2_t<float64_t> Point2d_t;
typedef Point2_t<float32_t> Point2f_t;
typedef Point2_t<int32_t>   Point2i_t;
typedef Point2_t<int16_t>   Point2s_t;
typedef Point2_t<int8_t>    Point2b_t;

/* For Point3*_t used in relative location:
 *     relLon: axis towards east, offset in longitude direction
 *     relLat: axis towards north, offset in latitude direction
 *     relAlt: axis towards up, offset in height direction
 *
 * For Point3*_t used in orientation:
 *     The orientation of vector starting from (0,0,0) to (relLon,relLat,relAlt)
 *     For the coordinate, refer to the definition in Point3*_t
 */

/***********************************************
 * Lines
 ***********************************************/
enum LINE_TYPE_E : uint8_t
{
    LINE_TYPE_SIMPLE_LINE_E = 0,
    LINE_TYPE_POLY3_E,     //control point is range of t
    LINE_TYPE_POLY3_CTRLPNT3D_E, //3d control point
    LINE_TYPE_SPLINE_E,
    LINE_TYPE_NURBS_E,
    LINE_TYPE_MAX_E,
};

//EQUATION_TYPE_independentVarialbe_dependentVariable_maxpower
enum EQUATION_TYPE_E : uint8_t
{
    EQUATION_TYPE_X_YZ_2_E = 0,   //independent variable x; dependent varialbe y, z; max power 2;
    EQUATION_TYPE_Y_XZ_2_E,       //independent variable y; dependent varialbe x, z; max power 2;
    EQUATION_TYPE_Z_XY_2_E,       //independent variable z; dependent varialbe x, y; max power 2;
    EQUATION_TYPE_X_YZ_3_E,       //independent variable x; dependent varialbe y, z; max power 3;
    EQUATION_TYPE_Y_XZ_3_E,       //independent variable y; dependent varialbe x, z; max power 3;
    EQUATION_TYPE_Z_XY_3_E,       //independent variable z; dependent varialbe x, y; max power 3;
    EQUATION_TYPE_MAX_E
};

struct Line_t: public Geometry_t
{
    LINE_TYPE_E eLineType;

    Line_t(LINE_TYPE_E eInLineType): Geometry_t(GEOMETRY_TYPE_LINE_E), eLineType(eInLineType) {}
};

struct SimpleLine_t: public Line_t
{
    std::vector<std::shared_ptr<Point_t>> vecPoint; // The ePointType field in Point_t should be same for all points

    SimpleLine_t(): Line_t(LINE_TYPE_SIMPLE_LINE_E) {}
};

struct Poly3Parameters_t
{
    std::vector<float64_t>  vecContPointPair; // the line (or paint) start / end pairs
    float64_t  equationCoef_x[4];   // x^0, x^1, x^2, x^3
    float64_t  equationCoef_y[4];
    float64_t  equationCoef_z[4];

    Poly3Parameters_t()
    {
        memset(equationCoef_x, 0, sizeof(equationCoef_x));
        memset(equationCoef_y, 0, sizeof(equationCoef_y));
        memset(equationCoef_z, 0, sizeof(equationCoef_z));
    }
};

struct Poly3CtrlPnt3DParam_t
{
    EQUATION_TYPE_E equationType;
    ObserveKfInfo_t observeKf;
    LANE_MARKING_TYPE_E subBoundaryType;

    /*Support up to 256 */
    std::vector<std::shared_ptr<Point_t>> ctrlPnts; // control points

    /*Support up to 256 */
    std::vector<float64_t> xCoefs; //sorted by descending power
    std::vector<float64_t> yCoefs; //sorted by descending power
    std::vector<float64_t> zCoefs; //sorted by descending power

    Poly3CtrlPnt3DParam_t(): equationType(EQUATION_TYPE_MAX_E), subBoundaryType(LANE_MARKING_TYPE_MAX_E) {}
};

struct PolyLine_t : public Line_t
{
    std::vector<Poly3Parameters_t> vecCurve;

    PolyLine_t(): Line_t(LINE_TYPE_POLY3_E) {}
};

/* Code 2-byte vector size */
struct PolyLineCtrlPnt3D_t : public Line_t
{
    std::vector<Poly3CtrlPnt3DParam_t> vecCurve;

    PolyLineCtrlPnt3D_t(): Line_t(LINE_TYPE_POLY3_CTRLPNT3D_E) {}
};

struct Spline_t : public Line_t
{
    //Spline():lineType(LINE_TYPE_SPLINE_E) {};
    // curve shape
    std::vector<Point3f_t> vecCtrlPoint; // size: N, where N < 256
    std::vector<float32_t> vecKnotVector; // size: N+P+1, P is degree number
    // paint
    std::vector<uint8_t>  vecPaintSEIdx; // the index in vecCtrlPoint for paint start / end pairs

    Spline_t(): Line_t(LINE_TYPE_SPLINE_E) {}
};

struct NURBS_t : public Line_t
{
    std::vector<Point3f_t> vecCtrlPoint;
    std::vector<float32_t> vecKnot;
    std::vector<float32_t> endPoint;

    uint32_t               degree;
    float32_t              paintTotalLength;
    float32_t              lineLength;

    NURBS_t() : Line_t(LINE_TYPE_NURBS_E), degree(2), paintTotalLength(0), lineLength(0)
    {
    }
};

/***********************************************
 * Surfaces
 ***********************************************/
enum SURFACE_TYPE_E : uint8_t
{
    SURFACE_TYPE_SIMPLE_E = 0,
    SURFACE_TYPE_BELT_E,
    SURFACE_TYPE_WALL_E,
    SURFACE_TYPE_RECTANGLE_E,
    SURFACE_TYPE_ELLIPSE_E,
    SURFACE_TYPE_POLYGON_E,
    SURFACE_TYPE_MAX_E
};

struct Surface_t : public Geometry_t
{
    SURFACE_TYPE_E eSurfaceType;

    Surface_t(SURFACE_TYPE_E eInSurfaceType):
                  Geometry_t(GEOMETRY_TYPE_SURFACE_E),
                  eSurfaceType(eInSurfaceType)
    {
    }
};

struct SimpleSurface_t : public Surface_t
{
    Point3f_t normalVector; // normal vector start from (0,0,0)

    SimpleSurface_t(): Surface_t(SURFACE_TYPE_SIMPLE_E) {}
};

struct BeltSurface_t : public Surface_t
{
    std::shared_ptr<Line_t> centerLine;
    float32_t width;
    uint8_t   colour[3];

    BeltSurface_t(): Surface_t(SURFACE_TYPE_BELT_E), width(0)
    {
        memset(colour, 0, sizeof(colour));
    }
};

struct WallSurface_t : public Surface_t
{
    std::shared_ptr<Line_t> bottomLine;
    float32_t height;

    WallSurface_t(): Surface_t(SURFACE_TYPE_WALL_E), height(0) {}
};

struct Rectangle_t : public Surface_t
{
    Point3f_t point1;
    Point3f_t point2;
    Point3f_t point3;

    Rectangle_t(): Surface_t(SURFACE_TYPE_RECTANGLE_E) {}
};

struct Ellipse_t : public Surface_t
{
    Point3f_t center;
    Point3b_t orientation;  // The direction of long axis pointing from center to this point
    float32_t longAxisLen;  // in meter
    float32_t shortAxisLen; // in meter

    Ellipse_t(): Surface_t(SURFACE_TYPE_ELLIPSE_E), longAxisLen(0.0), shortAxisLen(0.0) {}
};

struct  Polygon_t : public Surface_t
{
    Point3b_t               orientation;
    std::vector<Point3f_t>  location; // from top-left, clockwise, all points on a plane

    Polygon_t(): Surface_t(SURFACE_TYPE_POLYGON_E) {}
};

/***********************************************
 * Volumes
 ***********************************************/
enum VOLUME_TYPE_E : uint8_t
{
    VOLUME_TYPE_SIMPLE_E = 0,
    VOLUME_TYPE_CUBE_E,
    VOLUME_TYPE_POLE_E,
    VOLUME_TYPE_MAX_E
};

struct Volume_t : public Geometry_t
{
    VOLUME_TYPE_E eVolumeType;

    Volume_t(VOLUME_TYPE_E eInVolumeType): Geometry_t(GEOMETRY_TYPE_VOLUME_E), eVolumeType(eInVolumeType) {}
};

struct SimpleVolume_t : public Volume_t
{
    SimpleVolume_t(): Volume_t(VOLUME_TYPE_SIMPLE_E) {}

};

struct Cube_t : public Volume_t
{
    uint16_t w;    // width in meter, in Q(9,7) format
    uint16_t h;    // height in meter, in Q(9,7) format
    uint16_t d;    // depth in meter, in Q(9,7) format
    Point3b_t orientation; // the vector point from cube center to center of w-h
                           // surface and point against driver view.
    Point3f_t location; // relative location of cube center in meter

    Cube_t(): Volume_t(VOLUME_TYPE_CUBE_E), w(0), h(0), d(0) {}
};

struct Pole_t : public Volume_t
{
    uint16_t d;    // diameter in meter, in Q(9,7) format
    uint16_t h;    // height in meter, in Q(9,7) format
    Point3b_t orientation; // the vector in parallel of h and point up.
    Point3f_t location; // relative location of pole center in meter

    Pole_t(): Volume_t(VOLUME_TYPE_POLE_E), d(0), h(0) {}
};

template<>
struct Point2TypeTrais<float64_t>
{
    static const POINT_TYPE_E value = POINT_TYPE_E::POINT_TYPE_2D_E;
};

template<>
struct Point2TypeTrais<float32_t>
{
    static const POINT_TYPE_E value = POINT_TYPE_E::POINT_TYPE_2F_E;
};

template<>
struct Point2TypeTrais<int32_t>
{
    static const POINT_TYPE_E value = POINT_TYPE_2I_E;
};

template<>
struct Point2TypeTrais<int16_t>
{
    static const POINT_TYPE_E value = POINT_TYPE_2S_E;
};

template<>
struct Point2TypeTrais<int8_t>
{
    static const POINT_TYPE_E value = POINT_TYPE_2B_E;
};

template<>
struct Point3TypeTrais<float64_t>
{
    static const POINT_TYPE_E value = POINT_TYPE_3D_E;
};

template<>
struct Point3TypeTrais<float32_t>
{
    static const POINT_TYPE_E value = POINT_TYPE_3F_E;
};

template<>
struct Point3TypeTrais<int32_t>
{
    static const POINT_TYPE_E value = POINT_TYPE_3I_E;
};

template<>
struct Point3TypeTrais<int16_t>
{
    static const POINT_TYPE_E value = POINT_TYPE_3S_E;
};

template<>
struct Point3TypeTrais<int8_t>
{
    static const POINT_TYPE_E value = POINT_TYPE_3B_E;
};




}


#endif //RDB_V2S_GEOMETRY_H
