/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RdbServerSeriaImpRoad.h
 * @brief  Implementation of data serialization and deserialization for
 *         logicDB
 *******************************************************************************
 */
#include <memory>
#include "algoInterface/IRoad.h"
#include "serialization/rdb/RdbV2SBinSerializer.h"
#include "serialization/rdb/RdbV2SBinDeserializer.h"
#include "serialization/rdb/RdbV2SSeriaImpCommon.h"
#include "serialization/rdb/RdbV2SSeriaImpGeometry.h"

#ifndef RDB_SERVER_SERIA_IMP_ROAD_H
#define RDB_SERVER_SERIA_IMP_ROAD_H

namespace roadDBCore
{
namespace rdbSerialization
{

// Lane connection info
struct SLaneConnection_t
{
    uint64_t roadID;

    int32_t lLaneIndex;
    int32_t rLaneIndex;

    SLaneConnection_t():roadID(0), lLaneIndex(0), rLaneIndex(0)
    {
    }
};

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SLaneConnection_t data.
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
void serialize(Seria &seria, SLaneConnection_t &obj)
{
    seria & obj.roadID
          & obj.lLaneIndex
          & obj.rLaneIndex;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SCurve_t data.
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
void serialize(Seria &seria, SCurve_t &obj)
{
    seria & obj.dbID
          & obj.index
          & obj.type
          & obj.color
          & obj.width
          & obj.length
          & obj.equationDescription;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SLine_t data.
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
void serialize(Seria &seria, SLine_t &obj)
{
    seria & obj.dbID
          & obj.index
          & obj.confidence
          & obj.length
          & obj.curves;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SEVP_Attribute_t data.
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
void serialize(RdbV2SBinSerializer &seria, std::shared_ptr<SEVP_Attribute_t> &obj);
void serialize(RdbV2SBinDeserializer &seria, std::shared_ptr<SEVP_Attribute_t> &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SEVP_Attr_Lane_Change_t data.
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
void serialize(Seria &seria, SEVP_Attr_Lane_Change_t &obj)
{
    seria & obj.lCanChange
          & obj.rCanChange;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SEVP_Attr_Speed_Limit_t data.
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
void serialize(Seria &seria, SEVP_Attr_Speed_Limit_t &obj)
{
    seria & obj.index
          & obj.objType
          & obj.minSpeed
          & obj.maxSpeed;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SEVP_Attr_Turn_t data.
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
void serialize(Seria &seria, SEVP_Attr_Turn_t &obj)
{
    seria & obj.index
          & obj.objType
          & obj.turnType;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SEVP_Attr_Ts_t data.
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
void serialize(Seria &seria, SEVP_Attr_Ts_t &obj)
{
    seria & obj.index
          & obj.tsType;
}


/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SEVP_t data.
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
void serialize(Seria &seria, SEVP_t &obj)
{
    seria & obj.line
          & obj.vRTVInfo
          & obj.fromNodeDBID
          & obj.toNodeDBID
          & obj.type
          & obj.surfaceType
          & obj.vSegAttr
          & obj.passedVehicleNum;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SLane_t data.
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
void serialize(Seria &seria, SLane_t &obj)
{
    seria & obj.dbID
          & obj.isVirtual
          & obj.index
          & obj.leftLine
          & obj.centerLine
          & obj.rightLine
          & obj.evpLine;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SRoadEdge_t data.
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
void serialize(Seria &seria, SRoadEdge_t &obj)
{
    seria & obj.dbID
          & obj.index
          & obj.bLeft
          & obj.type
          & obj.height;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SRoadDiscreteObject_t data.
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
void serialize(Seria &seria, SRoadDiscreteObject_t &obj)
{
    seria & obj.dbID
          & obj.index
          & obj.segmentID
          & obj.type
          & obj.color
          & obj.locReliability
          & obj.semReliability
          & obj.orientation
          & obj.centerPoint
          & obj.model
          & obj.additionalInfos;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SRoadNode_t data.
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
void serialize(Seria &seria, SRoadNode_t &obj)
{
    seria & obj.dbID
          & obj.type
          & obj.position;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SRoad_t data.
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
void serialize(Seria &seria, SRoad_t &obj)
{
    seria & obj.dbID
          & obj.spFromNode
          & obj.spToNode
          & obj.length
          & obj.weight
          & obj.fixed
          & obj.divisionIDs
          & obj.lanes
          & obj.setPassedSeg
          & obj.roadEdges
          & obj.discreteObjects;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SIntersection_t data.
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
void serialize(Seria &seria, SIntersection_t &obj)
{
    seria & obj.dbID
          & obj.type
          & obj.roads
          & obj.boundary;
}

}
}
#endif //RDB_SERVER_SERIA_IMP_ROAD_H
