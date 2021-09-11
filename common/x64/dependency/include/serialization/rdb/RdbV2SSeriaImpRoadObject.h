/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RdbV2SSeriaImpRoadObject.h
 * @brief  Implementation of data serialization and deserialization for
 *         road object types
 *******************************************************************************
 */

#include "CommunicateDef/RdbV2SCommon.h"
#include "CommunicateDef/RdbV2SGeometry.h"
#include "CommunicateDef/RdbV2SRoadObject.h"
#include "serialization/rdb/RdbSeriaCommon.h"
#include "serialization/rdb/RdbV2SSeriaImpCommon.h"
#include "serialization/rdb/RdbV2SSeriaImpGeometry.h"
#include "serialization/rdb/RdbV2SBinSerializer.h"
#include "serialization/rdb/RdbV2SBinDeserializer.h"

#ifndef RDB_V2S_SERIA_IMP_ROAD_OBJECT_H
#define RDB_V2S_SERIA_IMP_ROAD_OBJECT_H

namespace roadDBCore
{

namespace rdbSerialization
{

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize std::shared_ptr<RoadObject_t> data.
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
void serialize(RdbV2SBinDeserializer &seria, std::shared_ptr<RoadObject_t> &obj);
void serialize(RdbV2SBinSerializer &seria, std::shared_ptr<RoadObject_t> &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize std::shared_ptr<Lane_t> data.
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
void serialize(RdbV2SBinSerializer &seria,  std::shared_ptr<Lane_t> &obj);
void serialize(RdbV2SBinDeserializer &seria,  std::shared_ptr<Lane_t> &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SimpleLane_t data.
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
void serialize(Seria &seria, SimpleLane_t &obj)
{
    seria & obj.isCurLane & obj.startLogicPointIndices
          & obj.endLogicPointIndices & obj.laneWidths
          & obj.lLine & obj.rLine;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize LaneLineSeg_t data.
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
void serialize(RdbV2SBinSerializer &seria, LaneLineSeg_t &obj);

template<>
void serialize(RdbV2SBinDeserializer &seria, LaneLineSeg_t &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize LaneLine_t data.
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
void serialize(Seria &seria, LaneLine_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, LaneLineSeg_t> cvecLineSegs(obj.vecLineSegs);
    seria & cvecLineSegs;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize std::shared_ptr<LaneBoundary_t> data.
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
void serialize(RdbV2SBinDeserializer &seria, std::shared_ptr<LaneBoundary_t> &obj);
void serialize(RdbV2SBinSerializer &seria, std::shared_ptr<LaneBoundary_t> &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SimpleLaneBoundary_t data.
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
void serialize(Seria &seria, SimpleLaneBoundary_t &obj)
{
    seria & obj.index & obj.weight & obj.curve;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize LaneLogicInfo_t data.
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
void serialize(Seria &seria, LaneLogicInfo_t &obj)
{
    seria & obj.observeKfInfo & obj.leftLineIdx & obj.rightLineIdx ;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize std::shared_ptr<RoadSurfaceObject_t> data.
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
void serialize(RdbV2SBinDeserializer &seria, std::shared_ptr<RoadSurfaceObject_t> &obj);
void serialize(RdbV2SBinSerializer &seria, std::shared_ptr<RoadSurfaceObject_t> &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize PedCrossingPaint_t data.
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
void serialize(Seria &seria, PedCrossingPaint_t &obj)
{
//    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, std::shared_ptr<Geometry_t> > cvDes(obj.descriptor);

//    seria & obj.geometryInfo & obj.reliability & obj.observeKfInfo & cvDes;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize HaltLinePaint_t data.
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
void serialize(Seria &seria, HaltLinePaint_t &obj)
{
//    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, std::shared_ptr<Geometry_t> > cvDes(obj.descriptor);

//    seria & obj.geometryInfo & obj.reliability & obj.observeKfInfo & cvDes;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize TempPaint_t data.
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
void serialize(Seria &seria, TempPaint_t &obj)
{
//    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, std::shared_ptr<Geometry_t> > cvDes(obj.descriptor);

//    seria & obj.geometryInfo & obj.reliability & obj.observeKfInfo & cvDes;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize ExclusionPaint_t data.
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
void serialize(Seria &seria, ExclusionPaint_t &obj)
{
//    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, std::shared_ptr<Geometry_t> > cvDes(obj.descriptor);

//    seria & obj.geometryInfo & obj.reliability & obj.observeKfInfo & cvDes;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize std::shared_ptr<ArrowPaint_t> data.
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
void serialize(RdbV2SBinDeserializer &seria, std::shared_ptr<ArrowPaint_t> &obj);
void serialize(RdbV2SBinSerializer &seria, std::shared_ptr<ArrowPaint_t> &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize ByciclePaint_t data.
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
void serialize(Seria &seria, ByciclePaint_t &obj)
{
//    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, std::shared_ptr<Geometry_t> > cvDes(obj.descriptor);

//    seria & obj.geometryInfo & obj.reliability & obj.observeKfInfo & cvDes;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize BusPaint_t data.
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
void serialize(Seria &seria, BusPaint_t &obj)
{
//    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, std::shared_ptr<Geometry_t> > cvDes(obj.descriptor);

//    seria & obj.geometryInfo & obj.reliability & obj.observeKfInfo & cvDes;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize HOVPaint_t data.
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
void serialize(Seria &seria, HOVPaint_t &obj)
{
//    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, std::shared_ptr<Geometry_t> > cvDes(obj.descriptor);

//    seria & obj.geometryInfo & obj.reliability & obj.observeKfInfo & cvDes;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SpeedBump_t data.
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
void serialize(Seria &seria, SpeedBump_t &obj)
{
//    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, std::shared_ptr<Geometry_t> > cvDes(obj.descriptor);

//    seria & obj.geometryInfo & obj.reliability & obj.observeKfInfo & cvDes;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize Manhole_t data.
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
void serialize(Seria &seria, Manhole_t &obj)
{
//    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, std::shared_ptr<Geometry_t> > cvDes(obj.descriptor);

//    seria & obj.geometryInfo & obj.reliability & obj.observeKfInfo & cvDes;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize std::shared_ptr<RoadFurnitureObject_t> data.
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
void serialize(RdbV2SBinDeserializer &seria, std::shared_ptr<RoadFurnitureObject_t> &obj);
void serialize(RdbV2SBinSerializer &seria, std::shared_ptr<RoadFurnitureObject_t> &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize TrafficSign_t data.
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
void serialize(Seria &seria, TrafficSign_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, std::shared_ptr<Geometry_t> > cvDes(obj.descriptor);

    seria & obj.geometryInfo & obj.reliability & obj.observeKfInfo
       & cvDes & obj.trafficSignType;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize TrafficLight_t data.
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
void serialize(Seria &seria, TrafficLight_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, std::shared_ptr<Geometry_t> > cvDes(obj.descriptor);

    seria & obj.geometryInfo & obj.reliability & obj.observeKfInfo & cvDes;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize std::shared_ptr<RoadEdge_t> data.
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
void serialize(RdbV2SBinDeserializer &seria, std::shared_ptr<RoadEdge_t> &obj);
void serialize(RdbV2SBinSerializer &seria, std::shared_ptr<RoadEdge_t> &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize PavementEdge_t data.
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
void serialize(Seria &seria, PavementEdge_t &obj)
{
    seria & obj.geometryInfo & obj.reliability & obj.observeKfInfo & obj.bRightSide;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize Curb_t data.
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
void serialize(Seria &seria, Curb_t &obj)
{
    seria & obj.geometryInfo & obj.reliability & obj.observeKfInfo & obj.bRightSide;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize GuardRail_t data.
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
void serialize(Seria &seria, GuardRail_t &obj)
{
    seria & obj.geometryInfo & obj.reliability & obj.observeKfInfo & obj.bRightSide;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize JerseyWall_t data.
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
void serialize(Seria &seria, JerseyWall_t &obj)
{
    seria & obj.geometryInfo & obj.reliability & obj.observeKfInfo & obj.bRightSide;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize RTMatrixItemSR_t data.
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
void serialize(Seria &seria, RTMatrixItemSR_t &obj)
{
    seria & obj.kfIdx & obj.isKeyFrame & obj.quaternion & obj.translation;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize RTItemsInDivisionSR_t data.
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
void serialize(Seria &seria, RTItemsInDivisionSR_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, RTMatrixItemSR_t> cvecRTItems(obj.vecRTItems);

    seria & obj.divisionID & obj.referenceID & obj.version & obj.height & cvecRTItems;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize LogicPoint_t data.
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
void serialize(RdbV2SBinSerializer &seria, LogicPoint_t &obj);

template<>
void serialize(RdbV2SBinDeserializer &seria, LogicPoint_t &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize LineViewItem_t data.
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
void serialize(Seria &seria, LineViewItem_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, Point2f_t> cvec2dPaint(obj.vec2dPaint);

    seria & obj.lineId
          & cvec2dPaint;
}


/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize ViewInfoVehicle_t data.
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
void serialize(Seria &seria, ViewInfoVehicle_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, LineViewItem_t> cvec2dPaint(obj.vecLineViewItems);

    seria & cvec2dPaint;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize LANE_TYPE_E data.
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
void serialize(RdbV2SBinSerializer &seria, LANE_TYPE_E &obj);

template<>
void serialize(RdbV2SBinDeserializer &seria, LANE_TYPE_E &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize RoadGeometryPayload_t data.
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
void serialize(RdbV2SBinSerializer &seria, RoadGeometryPayload_t &obj);

template<>
void serialize(RdbV2SBinDeserializer &seria, RoadGeometryPayload_t &obj);




}// namespace rdbSerialization

}// namespace roadDBCore




#endif //RDB_V2S_SERIA_IMP_ROAD_OBJECT_H



