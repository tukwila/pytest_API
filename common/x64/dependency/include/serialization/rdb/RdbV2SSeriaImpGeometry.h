/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RdbV2SSeriaImpGeometry.h
 * @brief  Implementation of data serialization and deserialization for
 *         geometry type
 *******************************************************************************
 */

#include "CommunicateDef/RdbV2SCommon.h"
#include "CommunicateDef/RdbV2SGeometry.h"
#include "serialization/rdb/RdbSeriaCommon.h"
#include "serialization/rdb/RdbV2SSeriaImpCommon.h"
#include "serialization/rdb/RdbV2SBinSerializer.h"
#include "serialization/rdb/RdbV2SBinDeserializer.h"
#include "CommunicateDef/RdbV2SRoadObject.h"
#ifndef RDB_V2S_SERIA_IMP_GEOMETRY_H
#define RDB_V2S_SERIA_IMP_GEOMETRY_H

namespace roadDBCore
{

namespace rdbSerialization
{

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize std::shared_ptr<Geometry_t> data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
void serialize(RdbV2SBinDeserializer &seria, std::shared_ptr<Geometry_t> &obj);
void serialize(RdbV2SBinSerializer &seria, std::shared_ptr<Geometry_t> &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize std::shared_ptr<Point_t> data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
void serialize(RdbV2SBinDeserializer &seria, std::shared_ptr<Point_t> &obj);
void serialize(RdbV2SBinSerializer &seria, std::shared_ptr<Point_t> &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize Point2_t<T> data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria, typename T>
void serialize(Seria &seria, Point2_t<T> &obj)
{
    seria & obj.x & obj.y;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize Point3_t<T> data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria, typename T>
void serialize(Seria &seria, Point3_t<T> &obj)
{
    seria & obj.relLon & obj.relLat & obj.relAlt;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize std::shared_ptr<Line_t> data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
void serialize(RdbV2SBinDeserializer &seria, std::shared_ptr<Line_t> &obj);
void serialize(RdbV2SBinSerializer &seria, std::shared_ptr<Line_t> &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize NURBS_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, NURBS_t &obj)
{
    seria & obj.vecCtrlPoint 
          & obj.vecKnot
          & obj.endPoint 
          // In order not to invalidate the old snippets,
          // do not serialize `degree` at this moment
          // & obj.degree
          & obj.paintTotalLength
          & obj.lineLength;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SimpleLine_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<>
void serialize(RdbV2SBinDeserializer &seria, SimpleLine_t &obj);

template<>
void serialize(RdbV2SBinSerializer &seria, SimpleLine_t &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize Spline_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, Spline_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, Point3f_t > cvCtrlPoint(obj.vecCtrlPoint);
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, float32_t > cvKnotVector(obj.vecKnotVector);
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, uint8_t > cvPaintSEIdx(obj.vecPaintSEIdx);

    seria & cvCtrlPoint & cvKnotVector & cvPaintSEIdx;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize PolyLine_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, PolyLine_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, Poly3Parameters_t> cvCurve(obj.vecCurve);

    seria & cvCurve;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize Poly3Parameters_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, Poly3Parameters_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, float64_t > cvContPointPair(obj.vecContPointPair);

    seria & cvContPointPair
        & obj.equationCoef_x[0] & obj.equationCoef_x[1] & obj.equationCoef_x[2] & obj.equationCoef_x[3]
        & obj.equationCoef_y[0] & obj.equationCoef_y[1] & obj.equationCoef_y[2] & obj.equationCoef_y[3]
        & obj.equationCoef_z[0] & obj.equationCoef_z[1] & obj.equationCoef_z[2] & obj.equationCoef_z[3];
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize PolyLineCtrlPnt3D_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, PolyLineCtrlPnt3D_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, Poly3CtrlPnt3DParam_t> cvCurve(obj.vecCurve);

    seria & cvCurve;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize Poly3CtrlPnt3DParam_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, Poly3CtrlPnt3DParam_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_1BYTE_E, std::shared_ptr<Point_t> > ctrlPnts(obj.ctrlPnts);
    CodingVector<VECTOR_CODE_TYPE_1BYTE_E, float64_t> xCoefs(obj.xCoefs);
    CodingVector<VECTOR_CODE_TYPE_1BYTE_E, float64_t> yCoefs(obj.yCoefs);
    CodingVector<VECTOR_CODE_TYPE_1BYTE_E, float64_t> zCoefs(obj.zCoefs);

    seria & obj.equationType
          & obj.observeKf
          & obj.subBoundaryType
          & ctrlPnts
          & xCoefs
          & yCoefs
          & zCoefs;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize std::shared_ptr<Surface_t> data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
void serialize(RdbV2SBinDeserializer &seria, std::shared_ptr<Surface_t> &obj);
void serialize(RdbV2SBinSerializer &seria, std::shared_ptr<Surface_t> &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SimpleSurface_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, SimpleSurface_t &obj)
{
    seria & obj.normalVector;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize BeltSurface_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, BeltSurface_t &obj)
{
    seria & obj.centerLine & obj.width & obj.colour[0] & obj.colour[1] & obj.colour[2];
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize WallSurface_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, WallSurface_t &obj)
{
    seria & obj.bottomLine & obj.height;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize Rectangle_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, Rectangle_t &obj)
{
    seria & obj.point1 & obj.point2 & obj.point3;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize Ellipse_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, Ellipse_t &obj)
{
    seria & obj.center & obj.orientation & obj.longAxisLen & obj.shortAxisLen;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize Ellipse_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, Polygon_t &obj)
{
    seria & obj.orientation & obj.location;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize std::shared_ptr<Volume_t> data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
void serialize(RdbV2SBinDeserializer &seria, std::shared_ptr<Volume_t> &obj);
void serialize(RdbV2SBinSerializer &seria, std::shared_ptr<Volume_t> &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SimpleVolume_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, SimpleVolume_t &obj)
{
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize Cube_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, Cube_t &obj)
{
    seria & obj.w & obj.h & obj.d & obj.orientation & obj.location;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize Pole_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, Pole_t &obj)
{
    seria & obj.d & obj.h & obj.orientation & obj.location;
}


}// namespace rdbSerialization

}// namespace roadDBCore




#endif //RDB_V2S_SERIA_IMP_GEOMETRY_H



